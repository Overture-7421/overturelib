package com.overture.lib.subsystems.Swerve;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.overture.lib.motorcontrollers.OverTalonFX;
import com.overture.lib.sensors.OverCANCoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** The SwerveModule class */
public class SwerveModule {

  /** The ModuleConfig */
  private ModuleConfig config;

  /** The drive motor */
  private OverTalonFX driveMotor;

  /** The turn motor */
  private OverTalonFX turnMotor;

  /** The encoder */
  private OverCANCoder encoder;

  /** The feedforward controller */
  private SimpleMotorFeedforward feedForward;

  /** Target Module State */
  private SwerveModuleState targetState = new SwerveModuleState();

  /** latest Module State */
  private SwerveModuleState latestState = new SwerveModuleState();

  /** Latest Module Position */
  private SwerveModulePosition latestPosition = new SwerveModulePosition();

  /** Turn Voltage */
  private PositionVoltage turnVoltage = new PositionVoltage(Rotations.of(0));

  /** Drive Voltage */
  private VoltageOut driveVoltage = new VoltageOut(Volts.of(0));

  /**
   * Constructor for SwerveModule
   *
   * @param config - ModuleConfig
   */
  public SwerveModule(ModuleConfig config) {
    this.config = config;
    this.driveMotor = new OverTalonFX(config.driveMotorConfig, config.canBus);
    this.turnMotor = new OverTalonFX(config.turnMotorConfig, config.canBus);
    this.encoder = new OverCANCoder(config.encoderConfig, config.canBus);
    this.feedForward = config.feedforward;

    this.turnMotor.setContinuousWrap();
    this.turnMotor.setFusedCANCoder(this.encoder);
    this.turnMotor.setControl(
        turnVoltage.withPosition(Rotations.of(0)).withEnableFOC(config.turnMotorConfig.useFOC));

    this.driveMotor.setPosition(Rotations.of(0));

    this.turnMotor.setPositionUpdateFrequency(Hertz.of(200));
    this.encoder.getAbsolutePosition().setUpdateFrequency(Hertz.of(200));
    this.driveMotor.setVelocityUpdateFrequency(Hertz.of(200));

    // Set Gear Ratios
    this.turnMotor.setRotorToSensorRatio(config.turnGearRatio);
    this.driveMotor.setSensorToMechanism(
        config.driveGearRatio * config.wheelDiameter.magnitude() * Math.PI);
  }

  /**
   * Gets the latest state of the module
   *
   * @return The latest state of the module
   */
  public SwerveModuleState getState() {
    return latestState;
  }

  /**
   * Sets the target state of the module
   *
   * @param state The target state of the module
   */
  public void setState(SwerveModuleState state) {
    state.optimize(latestState.angle);
    targetState = state;

    turnMotor.setControl(
        turnVoltage
            .withPosition(state.angle.getMeasure())
            .withEnableFOC(config.turnMotorConfig.useFOC)
            .withSlot(0));

    driveMotor.setControl(
        driveVoltage
            .withOutput(feedForward.calculate(targetState.speedMetersPerSecond))
            .withEnableFOC(config.driveMotorConfig.useFOC));
  }

  /**
   * Gets the module position
   *
   * @return - Module position
   */
  public SwerveModulePosition getPosition() {
    return latestPosition;
  }

  /**
   * Sets the raw voltage speed
   *
   * @param volts - Voltage
   */
  public void setVoltageDrive(Voltage volts) {
    driveMotor.setControl(
        driveVoltage.withOutput(volts).withEnableFOC(config.driveMotorConfig.useFOC));
  }

  /**
   * Sets the raw voltage speed
   *
   * @return - Voltage
   */
  public Voltage getDriveVoltage() {
    return driveMotor.getMotorVoltage().getValue();
  }

  /** Shuffleboard Periodic */
  public void shuffleboardPeriodic() {
    SmartDashboard.putNumber(
        config.moduleName + "/Current Speed", latestState.speedMetersPerSecond);
    SmartDashboard.putNumber(config.moduleName + "/Target Angle", targetState.angle.getDegrees());
    SmartDashboard.putNumber(config.moduleName + "/Current Angle", latestState.angle.getDegrees());
    SmartDashboard.putNumber(config.moduleName + "/Distance Meters", latestPosition.distanceMeters);
  }

  /** Periodic */
  public void Periodic() {
    Angle angle = encoder.getAbsolutePosition().getValue();
    latestState.speedMetersPerSecond = driveMotor.getVelocity().getValueAsDouble();
    latestState.angle = Rotation2d.fromRotations(angle.baseUnitMagnitude());

    latestPosition.distanceMeters = driveMotor.getPosition().getValueAsDouble();
    latestPosition.angle = Rotation2d.fromRotations(angle.baseUnitMagnitude());
  }
}
