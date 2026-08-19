// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.subsystems.swerve;

import com.ctre.phoenix6.CANBus;
import com.overture.lib.motorcontrollers.ControllerNeutralMode;
import com.overture.lib.motorcontrollers.OverTalonFXConfig;
import com.overture.lib.sensors.CanCoderConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

/** Configuration bundle for a {@link SwerveModule}. */
public class SwerveModuleConfig {
  /** Configuration of the drive motor. */
  public OverTalonFXConfig DriveMotorConfig = driveInit();

  /** Configuration of the turn motor. */
  public OverTalonFXConfig TurnMotorConfig = turnInit();

  /** Configuration of the absolute encoder. */
  public CanCoderConfig EncoderConfig = new CanCoderConfig();

  /** Name used to key this module in telemetry. */
  public String ModuleName = "";

  /** CAN bus every device on this module lives on. */
  public CANBus CanBus = new CANBus("");

  /** Wheel diameter, in meters. */
  public double WheelDiameter = 0.1016;

  /** Gear ratio between the turn rotor and the module azimuth. */
  public double TurnGearRatio = 1.0;

  /** Gear ratio between the drive rotor and the wheel. */
  public double DriveGearRatio = 1.0;

  /** Drive feedforward, in volts per meter per second. */
  public SimpleMotorFeedforward FeedForward;

  /**
   * Constructs a SwerveModuleConfig.
   *
   * @param feedForward the drive feedforward
   */
  public SwerveModuleConfig(SimpleMotorFeedforward feedForward) {
    this.FeedForward = feedForward;
  }

  /**
   * Copy constructor.
   *
   * <p>The C++ took and stored this struct by value, so re-stamping one config between the four
   * {@code new SwerveModule(config)} calls produced four independent modules. Java hands out
   * references, so {@link SwerveModule} snapshots through this constructor; without it all four
   * modules would share one object and scale odometry with whichever wheel diameter was written
   * last.
   *
   * @param other the configuration to copy
   */
  public SwerveModuleConfig(SwerveModuleConfig other) {
    DriveMotorConfig = new OverTalonFXConfig(other.DriveMotorConfig);
    TurnMotorConfig = new OverTalonFXConfig(other.TurnMotorConfig);
    EncoderConfig = new CanCoderConfig(other.EncoderConfig);
    ModuleName = other.ModuleName;
    CanBus = other.CanBus;
    WheelDiameter = other.WheelDiameter;
    TurnGearRatio = other.TurnGearRatio;
    DriveGearRatio = other.DriveGearRatio;
    FeedForward =
        new SimpleMotorFeedforward(
            other.FeedForward.getKs(),
            other.FeedForward.getKv(),
            other.FeedForward.getKa(),
            other.FeedForward.getDt());
  }

  /**
   * Returns the default drive motor configuration.
   *
   * @return the drive motor configuration
   */
  public static OverTalonFXConfig driveInit() {
    OverTalonFXConfig config = new OverTalonFXConfig();

    config.NeutralMode = ControllerNeutralMode.Brake;

    config.CurrentLimit = 40.0;
    config.StatorCurrentLimit = 120.0;
    config.TriggerThreshold = 70.0;
    config.TriggerThresholdTime = 0.5;
    config.ClosedLoopRampRate = 0.0;
    config.OpenLoopRampRate = 0.25;
    return config;
  }

  /**
   * Returns the default turn motor configuration.
   *
   * @return the turn motor configuration
   */
  public static OverTalonFXConfig turnInit() {
    OverTalonFXConfig config = new OverTalonFXConfig();

    config.NeutralMode = ControllerNeutralMode.Coast;

    config.CurrentLimit = 40.0;
    config.StatorCurrentLimit = 80.0;
    config.TriggerThreshold = 80.0;
    config.TriggerThresholdTime = 0.2;
    config.ClosedLoopRampRate = 0.0;
    config.OpenLoopRampRate = 0.0;
    return config;
  }
}
