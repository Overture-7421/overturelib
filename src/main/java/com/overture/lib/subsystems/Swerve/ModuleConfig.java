package com.overture.lib.subsystems.Swerve;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.overture.lib.motorcontrollers.OverTalonFXConfig;
import com.overture.lib.sensors.OverCANCoderConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Distance;

/** The ModuleConfig class */
public class ModuleConfig {

  /** The drive motor config */
  public OverTalonFXConfig driveMotorConfig;
  /** The turn motor config */
  public OverTalonFXConfig turnMotorConfig;
  /** The encoder config */
  public OverCANCoderConfig encoderConfig;
  /** The module name */
  public String moduleName;
  /** The can bus */
  public String canBus;
  /** The wheel diameter */
  public Distance wheelDiameter;
  /** The turn gear ratio */
  public double turnGearRatio;
  /** The drive gear ratio */
  public double driveGearRatio;
  /** The feedforward */
  public SimpleMotorFeedforward feedforward;

  /** Constructor for ModuleConfig */
  public ModuleConfig() {
    driveMotorConfig = DriveInit();
    turnMotorConfig = TurnInit();
    encoderConfig = new OverCANCoderConfig();
    moduleName = "";
    canBus = "";
    wheelDiameter = Meters.of(0.1016);
    turnGearRatio = 1.0;
    driveGearRatio = 1.0;
    feedforward = new SimpleMotorFeedforward(0.0, 0.0, 0.0);
  }

  /**
   * @param driveMotorConfig the driveMotorConfig to set
   * @return the ModuleConfig
   */
  public ModuleConfig withDriveMotorConfig(OverTalonFXConfig driveMotorConfig) {
    this.driveMotorConfig = driveMotorConfig;
    return this;
  }

  /**
   * @param turnMotorConfig the turnMotorConfig to set
   * @return the ModuleConfig
   */
  public ModuleConfig withTurnMotorConfig(OverTalonFXConfig turnMotorConfig) {
    this.turnMotorConfig = turnMotorConfig;
    return this;
  }

  /**
   * @param encoderConfig the encoderConfig to set
   * @return the ModuleConfig
   */
  public ModuleConfig withEncoderConfig(OverCANCoderConfig encoderConfig) {
    this.encoderConfig = encoderConfig;
    return this;
  }

  /**
   * @param moduleName the moduleName to set
   * @return the ModuleConfig
   */
  public ModuleConfig withModuleName(String moduleName) {
    this.moduleName = moduleName;
    return this;
  }

  /**
   * @param canBus the canBus to set
   * @return the ModuleConfig
   */
  public ModuleConfig withCanBus(String canBus) {
    this.canBus = canBus;
    return this;
  }

  /**
   * @param wheelDiameter the wheelDiameter to set
   * @return the ModuleConfig
   */
  public ModuleConfig withWheelDiameter(Distance wheelDiameter) {
    this.wheelDiameter = wheelDiameter;
    return this;
  }

  /**
   * @param turnGearRatio the turnGearRatio to set
   * @return the ModuleConfig
   */
  public ModuleConfig withTurnGearRatio(double turnGearRatio) {
    this.turnGearRatio = turnGearRatio;
    return this;
  }

  /**
   * @param driveGearRatio the driveGearRatio to set
   * @return the ModuleConfig
   */
  public ModuleConfig withDriveGearRatio(double driveGearRatio) {
    this.driveGearRatio = driveGearRatio;
    return this;
  }

  /**
   * @param feedforward the feedforward to set
   * @return the ModuleConfig
   */
  public ModuleConfig withFeedforward(SimpleMotorFeedforward feedforward) {
    this.feedforward = feedforward;
    return this;
  }

  /**
   * @return the driveMotorConfig
   */
  public static OverTalonFXConfig DriveInit() {
    return new OverTalonFXConfig()
        .withNeutralMode(NeutralModeValue.Brake)
        .withCurrentLimit(90)
        .withStatorCurrentLimit(120)
        .withTriggerThreshold(0.0)
        .withTriggerThresholdTime(0.5)
        .withClosedLoopRampRate(0)
        .withOpenLoopRampRate(0.25);
  }

  /**
   * @return the turnMotorConfig
   */
  public static OverTalonFXConfig TurnInit() {
    return new OverTalonFXConfig()
        .withNeutralMode(NeutralModeValue.Coast)
        .withCurrentLimit(60)
        .withStatorCurrentLimit(80)
        .withTriggerThreshold(0.0)
        .withTriggerThresholdTime(0.2)
        .withClosedLoopRampRate(0)
        .withOpenLoopRampRate(0);
  }
}
