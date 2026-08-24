// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.subsystems.swerve;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.overture.lib.motorcontrollers.ControllerNeutralMode;
import com.overture.lib.motorcontrollers.OverTalonFX;
import com.overture.lib.sensors.OverCANCoder;
import com.overture.lib.utils.Logging;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedBattery;

/** A single swerve module: one drive motor, one turn motor and one absolute encoder. */
public class SwerveModule extends SubsystemBase {
  private final SwerveModuleConfig config;

  // Declaration of motor controllers
  private final OverTalonFX driveMotor;
  private final OverTalonFX turnMotor;

  // Declaration of sensors
  private final OverCANCoder canCoder;

  // FeedForward
  private final SimpleMotorFeedforward feedForward;

  // State
  private SwerveModuleState targetState = new SwerveModuleState();

  private final SwerveModuleState latestState = new SwerveModuleState();
  private final SwerveModulePosition latestPosition = new SwerveModulePosition();

  private final PositionVoltage turnVoltage = new PositionVoltage(0);
  private final VoltageOut driveVoltage = new VoltageOut(0);

  /**
   * Constructs a SwerveModule.
   *
   * @param config the configuration of the module
   */
  public SwerveModule(SwerveModuleConfig config) {
    // Snapshot, matching the C++ which took and stored this struct by value. WheelDiameter,
    // ModuleName, useFOC and NeutralMode are all read live every loop, so four modules built from
    // one re-stamped config must not end up sharing it.
    this.config = new SwerveModuleConfig(config);
    this.driveMotor = new OverTalonFX(this.config.DriveMotorConfig, this.config.CanBus);
    this.turnMotor = new OverTalonFX(this.config.TurnMotorConfig, this.config.CanBus);
    this.canCoder = new OverCANCoder(this.config.EncoderConfig, this.config.CanBus);
    this.feedForward = this.config.FeedForward;

    turnMotor.setContinuousWrap();
    turnMotor.setFusedCANCoder(this.config.EncoderConfig.CanCoderId);
    turnMotor.setControl(
        turnVoltage.withPosition(0).withEnableFOC(this.config.TurnMotorConfig.useFOC));

    driveMotor.setPosition(0);

    turnMotor.setPositionUpdateFrequency(200);
    canCoder.getPosition().setUpdateFrequency(200);
    driveMotor.setVelocityUpdateFrequency(200);

    // Set Gear Ratios
    turnMotor.setRotorToSensorRatio(this.config.TurnGearRatio);
    driveMotor.setSensorToMechanism(this.config.DriveGearRatio);
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
   * @param state the desired state of the module
   */
  public void setState(SwerveModuleState state) {
    state.optimize(latestState.angle);
    state.cosineScale(latestState.angle);

    targetState = state;

    turnMotor.setControl(
        turnVoltage
            .withPosition(targetState.angle.getRotations())
            .withEnableFOC(config.TurnMotorConfig.useFOC)
            .withSlot(0));
    driveMotor.setControl(
        driveVoltage
            .withOutput(feedForward.calculate(targetState.speedMetersPerSecond))
            .withEnableFOC(config.DriveMotorConfig.useFOC));
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
    driveMotor.setControl(
        driveVoltage.withOutput(volts).withEnableFOC(config.DriveMotorConfig.useFOC));
  }

  /**
   * Returns the voltage currently applied to the drive motor.
   *
   * @return the drive voltage, in volts
   */
  public double getVoltageDrive() {
    return driveMotor.getMotorVoltage().getValueAsDouble();
  }

  /**
   * Sets the neutral mode of the drive motor.
   *
   * @param mode the neutral mode to apply
   */
  public void setDriveNeutralMode(ControllerNeutralMode mode) {
    driveMotor.setNeutralMode(
        mode == ControllerNeutralMode.Brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  /** Restores the drive motor to the neutral mode it was configured with. */
  public void restoreDriveNeutralMode() {
    setDriveNeutralMode(config.DriveMotorConfig.NeutralMode);
  }

  /**
   * Wires this module's motors to a simulated one, so the physics drives the encoders and the
   * motors' own closed loops drive the physics.
   *
   * <p>maple-sim asks a controller for the voltage it is commanding and hands back where the
   * mechanism actually ended up. Answering with the real {@link
   * com.ctre.phoenix6.sim.TalonFXSimState} means the control loops under test are the ones that run
   * on the robot, rather than a stand-in PID controller that only behaves similarly.
   *
   * <p>Every value crossing this boundary is a unit-carrying Measure, not a bare double. The
   * Phoenix simulation setters take rotations in some places and degrees in others, and passing
   * doubles is how a heading ends up multiplied by 360.
   *
   * @param moduleSim the simulated module to attach to
   */
  public void attachSimulation(SwerveModuleSimulation moduleSim) {
    driveMotor.getSimState().Orientation = simOrientation(config.DriveMotorConfig.Inverted);
    turnMotor.getSimState().Orientation = simOrientation(config.TurnMotorConfig.Inverted);

    // Phoenix subtracts this from whatever we hand setRawPosition below, cancelling the magnet
    // offset the encoder applies on the way out. Without it every module reads its true angle plus
    // its calibration, so the modules point wrong by the four numbers a team measured on the real
    // robot -- in simulation only, which invites zeroing the offsets for simulation and calling it
    // fixed.
    canCoder.getSimState().SensorOffset = canCoder.getConfiguration().MagnetSensor.MagnetOffset;

    moduleSim.useDriveMotorController(
        (mechanismAngle, mechanismVelocity, encoderAngle, encoderVelocity) -> {
          var sim = driveMotor.getSimState();
          sim.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
          sim.setRawRotorPosition(encoderAngle);
          sim.setRotorVelocity(encoderVelocity);
          return sim.getMotorVoltageMeasure();
        });

    moduleSim.useSteerMotorController(
        (mechanismAngle, mechanismVelocity, encoderAngle, encoderVelocity) -> {
          // The CANcoder is fused to the turn motor and reads the module's true facing, so it gets
          // the mechanism side of the gearbox while the motor gets the rotor side.
          var encoderSim = canCoder.getSimState();
          encoderSim.setRawPosition(mechanismAngle);
          encoderSim.setVelocity(mechanismVelocity);

          var sim = turnMotor.getSimState();
          sim.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
          sim.setRawRotorPosition(encoderAngle);
          sim.setRotorVelocity(encoderVelocity);
          return sim.getMotorVoltageMeasure();
        });
  }

  /**
   * Maps a motor's software inversion onto its physical orientation for the simulation.
   *
   * <p>Phoenix documents Orientation as describing the mechanical linkage and warns against setting
   * it from the invert. The two are the same fact on a robot configured the usual way: a module is
   * marked inverted precisely because its motor drives the wheel backwards. Tying them together
   * beats asking for the same fact twice, which is how the two answers end up disagreeing.
   */
  private static ChassisReference simOrientation(boolean inverted) {
    return inverted
        ? ChassisReference.Clockwise_Positive
        : ChassisReference.CounterClockwise_Positive;
  }

  /**
   * Publishes module telemetry, recorded and live.
   *
   * <p>Speed and angle travel together as a SwerveModuleState rather than as four loose numbers,
   * which halves the signal count and lets AdvantageScope draw the module instead of plotting it.
   */
  public void shuffleboardPeriodic() {
    String base = "/Swerve/Modules/" + config.ModuleName;

    Logging.logStruct(
        base + "/TargetState", SwerveModuleState.struct, targetState, Logging.Destination.BOTH);
    Logging.logStruct(
        base + "/State", SwerveModuleState.struct, latestState, Logging.Destination.BOTH);

    Logging.logDouble(
        base + "/Distance", latestPosition.distanceMeters, "m", Logging.Destination.LOG_ONLY);
    Logging.logDouble(
        base + "/RequestedVoltage",
        feedForward.calculate(targetState.speedMetersPerSecond),
        "V",
        Logging.Destination.LOG_ONLY);
    Logging.logDouble(
        base + "/AppliedVoltage",
        driveMotor.getMotorVoltage().getValueAsDouble(),
        "V",
        Logging.Destination.LOG_ONLY);
  }

  @Override
  public void periodic() {
    Rotation2d angle = Rotation2d.fromRotations(canCoder.getAbsolutePosition().getValueAsDouble());

    latestState.speedMetersPerSecond =
        driveMotor.getVelocity().getValueAsDouble() * config.WheelDiameter * Math.PI;
    latestState.angle = angle;

    latestPosition.distanceMeters =
        driveMotor.getPosition().getValueAsDouble() * config.WheelDiameter * Math.PI;
    latestPosition.angle = angle;
  }
}
