// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.overture.lib.motorcontrollers.OverTalonFX;
import com.overture.lib.sensors.OverCANCoder;
import com.overture.lib.utils.Logging;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** A single swerve module: one drive motor, one turn motor and one absolute encoder. */
public class SwerveModule extends SubsystemBase {
  private final SwerveModuleConfig config;

  // Declaration of motor controllers
  private final OverTalonFX driveMotor;
  private final OverTalonFX turnMotor;

  // Declaration of sensors
  private final OverCANCoder canCoder;

  // Sensor signals, looked up once. A device getter such as getPosition() looks its signal up again
  // and refreshes it with a native call every time it is called; these are refreshed in one batch
  // instead, by the chassis or by updateInputs().
  private final StatusSignal<Angle> drivePositionSignal;
  private final StatusSignal<AngularVelocity> driveVelocitySignal;
  private final StatusSignal<Voltage> driveVoltageSignal;
  private final StatusSignal<Double> driveFeedForwardSignal;
  private final StatusSignal<Angle> turnAngleSignal;

  // The five above, for BaseStatusSignal.refreshAll. All on config.CanBus, as refreshAll requires.
  private final BaseStatusSignal[] inputSignals;

  // Set once a chassis has taken over refreshing this module's sensors.
  private boolean updatedByChassis = false;

  // State
  private SwerveModuleState targetState = new SwerveModuleState();

  private final SwerveModuleState latestState = new SwerveModuleState();
  private final SwerveModulePosition latestPosition = new SwerveModulePosition();

  private final PositionVoltage turnVoltage = new PositionVoltage(0);
  private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

  // Both still used: driveVoltage by setVoltageDrive, which SysId drives directly, and
  // turnVoltageOut to park the azimuth inside the deadband.
  private final VoltageOut driveVoltage = new VoltageOut(0);
  private final VoltageOut turnVoltageOut = new VoltageOut(0);

  // Static-brakes the drive motor while its output is neutral, whatever its neutral mode config.
  private boolean driveBrakeOverride = false;

  // The wheel speed of the last setState and when it was asked for, so the next one can derive the
  // acceleration the Talon's kA acts on.
  private double lastWheelRotationsPerSecond = 0.0;
  private double lastSetStateTimestamp = 0.0;

  // Telemetry paths, built once instead of concatenated every loop.
  private final String feedForwardVoltagePath;
  private final String appliedVoltagePath;

  /**
   * Constructs a SwerveModule.
   *
   * @param config the configuration of the module
   */
  public SwerveModule(SwerveModuleConfig config) {

    this.config = new SwerveModuleConfig(config);

    this.config.TurnMotorConfig.ClosedLoopGeneral.withContinuousWrap(true);
    this.config.TurnMotorConfig.Feedback.withFeedbackRemoteSensorID(
            this.config.EncoderConfig.CanCoderId)
        .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
        .withRotorToSensorRatio(this.config.TurnGearRatio);
    this.config.DriveMotorConfig.Feedback.withSensorToMechanismRatio(this.config.DriveGearRatio);

    this.driveMotor =
        new OverTalonFX(this.config.DriveMotorConfig, this.config.driveMotorId, this.config.CanBus);
    this.turnMotor =
        new OverTalonFX(this.config.TurnMotorConfig, this.config.turnMotorId, this.config.CanBus);
    this.canCoder = new OverCANCoder(this.config.EncoderConfig, this.config.CanBus);

    turnMotor.setControl(turnVoltage.withPosition(0).withEnableFOC(this.config.useFOCTurn));

    driveMotor.setPosition(0);

    turnMotor.setPositionUpdateFrequency(200);
    canCoder.getPosition().setUpdateFrequency(200);
    driveMotor.setVelocityUpdateFrequency(200);

    drivePositionSignal = driveMotor.getPosition(false);
    driveVelocitySignal = driveMotor.getVelocity(false);
    driveVoltageSignal = driveMotor.getMotorVoltage(false);
    driveFeedForwardSignal = driveMotor.getClosedLoopFeedForward(false);
    turnAngleSignal = canCoder.getAbsolutePosition(false);
    inputSignals =
        new BaseStatusSignal[] {
          drivePositionSignal,
          driveVelocitySignal,
          driveVoltageSignal,
          driveFeedForwardSignal,
          turnAngleSignal
        };

    // Only logged, so the loop rate is plenty; the closed loop frames default to much slower.
    driveFeedForwardSignal.setUpdateFrequency(50);

    String base = "/Swerve/Modules/" + this.config.ModuleName;
    feedForwardVoltagePath = base + "/FeedForwardVoltage";
    appliedVoltagePath = base + "/AppliedVoltage";
  }

  /**
   * Gets the state of the module.
   *
   * @return the state of the module
   */
  public SwerveModuleState getState() {
    return latestState;
  }

  /**
   * Sets the state of the module.
   *
   * <p>The drive motor runs the Talon's own velocity loop: kS, kV and kA feed forward and kP
   * corrects the error, all from {@code DriveMotorConfig.Slot0} and all in wheel rotations per
   * second. Open loop meant carpet, a sagging battery and the robot's own weight all came out as
   * speed error that nothing corrected, and the four modules erred by different amounts, so the
   * chassis translated slower than asked and pulled off heading. A kP of zero is that behaviour
   * again.
   *
   * <p>The acceleration kA acts on is the change in the requested wheel speed since the last call,
   * so a request that steps shows up as a voltage spike for one loop; a drive command that limits
   * its own acceleration keeps it honest. It is skipped on the loop the request changes sign, where
   * {@code optimize} may have flipped the wheel rather than the robot.
   *
   * @param state the desired state of the module
   */
  public void setState(SwerveModuleState state) {
    state.optimize(latestState.angle);
    state.cosineScale(latestState.angle);

    targetState = state;

    // Close enough is left alone. Holding a position loop against the last fraction of a degree
    // only feeds encoder noise back into the azimuth, which is what is heard as the modules
    // buzzing while the robot stands still.
    double azimuthErrorDegrees = Math.abs(targetState.angle.minus(latestState.angle).getDegrees());

    if (azimuthErrorDegrees <= config.TurnDeadbandDegrees) {
      turnMotor.setControl(turnVoltageOut.withOutput(0.0));
    } else {
      turnMotor.setControl(
          turnVoltage
              .withPosition(targetState.angle.getRotations())
              .withEnableFOC(config.useFOCTurn)
              .withSlot(0));
    }

    // The drive motor's sensor is scaled to the mechanism, so its velocity is wheel rotations per
    // second, not rotor.
    double wheelRotationsPerSecond =
        targetState.speedMetersPerSecond / (Math.PI * config.WheelDiameter);

    double now = Timer.getFPGATimestamp();
    double dt = now - lastSetStateTimestamp;
    double wheelAcceleration = 0.0;
    if (dt > 0.0 && dt < 0.5 && wheelRotationsPerSecond * lastWheelRotationsPerSecond >= 0.0) {
      wheelAcceleration = (wheelRotationsPerSecond - lastWheelRotationsPerSecond) / dt;
    }
    lastSetStateTimestamp = now;
    lastWheelRotationsPerSecond = wheelRotationsPerSecond;

    driveMotor.setControl(
        driveVelocity
            .withVelocity(wheelRotationsPerSecond)
            .withAcceleration(wheelAcceleration)
            .withEnableFOC(config.useFOCDrive)
            .withOverrideBrakeDurNeutral(driveBrakeOverride)
            .withSlot(0));
  }

  /**
   * Gets the module position.
   *
   * @return the module position
   */
  public SwerveModulePosition getPosition() {
    return latestPosition;
  }

  /**
   * Sets the raw drive voltage.
   *
   * @param volts the voltage to apply
   */
  public void setVoltageDrive(double volts) {
    driveMotor.setControl(driveVoltage.withOutput(volts).withEnableFOC(config.useFOCDrive));
  }

  /**
   * Returns the voltage applied to the drive motor as of the last sensor refresh, which a chassis
   * does at the start of every loop.
   *
   * @return the drive voltage, in volts
   */
  public double getVoltageDrive() {
    return driveVoltageSignal.getValueAsDouble();
  }

  /**
   * Sets the neutral mode of the drive motor.
   *
   * <p>This writes the motor's configuration, which blocks until the motor acknowledges it. Keep it
   * out of anything that runs every loop.
   *
   * @param mode the neutral mode to apply
   */
  public void setDriveNeutralMode(NeutralModeValue mode) {
    driveMotor.configureNeutralMode(mode);
  }

  /**
   * Restores the drive motor to the neutral mode it was configured with. Blocks like {@link
   * #setDriveNeutralMode(NeutralModeValue)}.
   */
  public void restoreDriveNeutralMode() {
    driveMotor.configureNeutralMode(config.DriveMotorConfig.MotorOutput.NeutralMode);
  }

  /**
   * Static-brakes the drive motor whenever its output is neutral, whatever its neutral mode is
   * configured to.
   *
   * <p>This is how X mode holds the robot. It used to switch the neutral mode config on all four
   * drive motors, and a config write blocks the loop until the motor acknowledges it, so every
   * toggle cost four of those. This rides on the drive request instead and takes effect with the
   * next {@link #setState(SwerveModuleState)}.
   *
   * @param brake whether to brake while the output is neutral
   */
  void setDriveBrakeOverride(boolean brake) {
    driveBrakeOverride = brake;
  }

  /**
   * Returns the state the module was last asked for, after optimizing and cosine scaling.
   *
   * @return the target state
   */
  public SwerveModuleState getTargetState() {
    return targetState;
  }

  /**
   * Publishes what only this module knows: the feedforward volts the Talon computed from its gains
   * and the voltage it applied. The difference between the two is what kP contributed.
   *
   * <p>Its measured and target states and its distance are logged by the chassis, as the
   * /Swerve/Modules/States, TargetStates and Positions arrays (front left, front right, back left,
   * back right). Logging them here as well wrote every value twice.
   */
  public void shuffleboardPeriodic() {
    Logging.logDouble(
        feedForwardVoltagePath,
        driveFeedForwardSignal.getValueAsDouble(),
        "V",
        Logging.Destination.LOG_ONLY);
    Logging.logDouble(appliedVoltagePath, getVoltageDrive(), "V", Logging.Destination.LOG_ONLY);
  }

  /**
   * Refreshes this module's sensors and reads them into the cached state that {@link #getState()}
   * and {@link #getPosition()} serve.
   *
   * <p>A chassis does not call this. It refreshes all four modules in one batch and then reads each
   * one, which is the same work in one native call instead of one per signal.
   */
  public void updateInputs() {
    BaseStatusSignal.refreshAll(inputSignals);
    readInputs();
  }

  /**
   * Hands this module's sensors to a chassis, which refreshes them along with every other module's
   * and then calls {@link #readInputs()}. From then on {@link #periodic()} leaves them alone.
   *
   * @return the signals to refresh, all on {@link #getCanBus()}
   */
  BaseStatusSignal[] handInputsToChassis() {
    updatedByChassis = true;
    return inputSignals;
  }

  /**
   * Returns the CAN bus all of this module's devices are on.
   *
   * @return the module's CAN bus
   */
  CANBus getCanBus() {
    return config.CanBus;
  }

  /**
   * Reads the already refreshed sensors into the cached state and position. No native calls: a
   * signal's value is a plain field once it has been refreshed.
   */
  void readInputs() {
    Rotation2d angle = Rotation2d.fromRotations(turnAngleSignal.getValueAsDouble());

    latestState.speedMetersPerSecond =
        driveVelocitySignal.getValueAsDouble() * config.WheelDiameter * Math.PI;
    latestState.angle = angle;

    latestPosition.distanceMeters =
        drivePositionSignal.getValueAsDouble() * config.WheelDiameter * Math.PI;
    latestPosition.angle = angle;
  }

  @Override
  public void periodic() {
    // A chassis has already refreshed and read this module by the time this runs, as its first
    // step every loop. Reading again here used to cost three more native calls per module per loop
    // for the values the chassis had just read. A module built outside a chassis still updates
    // itself.
    if (!updatedByChassis) {
      updateInputs();
    }
  }
}
