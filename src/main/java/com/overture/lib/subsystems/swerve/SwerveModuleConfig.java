// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.subsystems.swerve;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.overture.lib.sensors.CanCoderConfig;

/** Configuration bundle for a {@link SwerveModule}. */
public class SwerveModuleConfig {
  /**
   * Configuration of the drive motor.
   *
   * <p>The drive gains live in its {@code Slot0}, in the Talon's own units: wheel rotations per
   * second, because {@link SwerveModule} scales the drive sensor to the wheel by {@link
   * #DriveGearRatio}. A gain tuned in meters per second converts by multiplying by the wheel
   * circumference, {@code Math.PI * WheelDiameter}. kS and kV are what makes the wheel turn at all,
   * kP corrects the speed error the Talon measures, and kA acts on the acceleration of the request,
   * which {@link SwerveModule#setState} derives from one loop to the next.
   */
  public TalonFXConfiguration DriveMotorConfig = driveInit();

  /** Configuration of the turn motor. */
  public TalonFXConfiguration TurnMotorConfig = turnInit();

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

  /** CAN id of the drive motor. */
  public int driveMotorId;

  /** CAN id of the turn motor. */
  public int turnMotorId;

  /** Whether the drive motor's control requests use Field Oriented Control. */
  public boolean useFOCDrive = false;

  /** Whether the turn motor's control requests use Field Oriented Control. */
  public boolean useFOCTurn = false;

  /**
   * How close the azimuth has to be, in degrees, before the turn motor is left alone.
   *
   * <p>Inside this band the module is commanded zero volts instead of holding a position loop, so
   * it stops chasing encoder noise while the robot is standing still. Zero disables it and restores
   * a loop that is always active.
   */
  public double TurnDeadbandDegrees = 0.5;

  /**
   * Constructs a SwerveModuleConfig.
   *
   * @param driveMotorId the CAN id of the drive motor
   * @param turnMotorId the CAN id of the turn motor
   */
  public SwerveModuleConfig(int driveMotorId, int turnMotorId) {
    this.driveMotorId = driveMotorId;
    this.turnMotorId = turnMotorId;
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
    DriveMotorConfig = other.DriveMotorConfig.clone();
    TurnMotorConfig = other.TurnMotorConfig.clone();
    EncoderConfig = new CanCoderConfig(other.EncoderConfig);
    ModuleName = other.ModuleName;
    CanBus = other.CanBus;
    WheelDiameter = other.WheelDiameter;
    TurnGearRatio = other.TurnGearRatio;
    DriveGearRatio = other.DriveGearRatio;
    TurnDeadbandDegrees = other.TurnDeadbandDegrees;
    this.driveMotorId = other.driveMotorId;
    this.turnMotorId = other.turnMotorId;
    this.useFOCDrive = other.useFOCDrive;
    this.useFOCTurn = other.useFOCTurn;
  }

  /**
   * Returns the default drive motor configuration.
   *
   * @return the drive motor configuration
   */
  public static TalonFXConfiguration driveInit() {
    return new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(120.0)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(70.0)
                .withSupplyCurrentLowerLimit(40)
                .withSupplyCurrentLowerTime(0.5))
        .withVoltage(
            new VoltageConfigs().withPeakForwardVoltage(12.0).withPeakReverseVoltage(-12.0))
        // Open loop only, so it softens setVoltageDrive and SysId without slowing the closed
        // velocity loop that setState drives.
        .withOpenLoopRamps(new OpenLoopRampsConfigs().withVoltageOpenLoopRampPeriod(0.25))
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake));
  }

  /**
   * Returns the default turn motor configuration.
   *
   * @return the turn motor configuration
   */
  public static TalonFXConfiguration turnInit() {
    return new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(80.0)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(80.0)
                .withSupplyCurrentLowerLimit(40)
                .withSupplyCurrentLowerTime(0.2))
        .withVoltage(
            new VoltageConfigs().withPeakForwardVoltage(12.0).withPeakReverseVoltage(-12.0))
        .withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast));
  }
}
