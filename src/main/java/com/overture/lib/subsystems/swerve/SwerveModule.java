// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.subsystems.swerve;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.overture.lib.motorcontrollers.ControllerNeutralMode;
import com.overture.lib.motorcontrollers.OverTalonFX;
import com.overture.lib.sensors.OverCANCoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

  /** Publishes module telemetry. */
  public void shuffleboardPeriodic() {
    String base = "SwerveChassis/Modules/" + config.ModuleName;
    SmartDashboard.putNumber(base + "/TargetSpeed", targetState.speedMetersPerSecond);
    SmartDashboard.putNumber(base + "/Speed", latestState.speedMetersPerSecond);
    SmartDashboard.putNumber(base + "/TargetAngle", targetState.angle.getDegrees());
    SmartDashboard.putNumber(base + "/Angle", latestState.angle.getDegrees());
    SmartDashboard.putNumber(base + "/Distance", latestPosition.distanceMeters);
    SmartDashboard.putNumber(
        base + "/RequestedVoltage", feedForward.calculate(targetState.speedMetersPerSecond));
    SmartDashboard.putNumber(
        base + "/AppliedVoltage", driveMotor.getMotorVoltage().getValueAsDouble());
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
