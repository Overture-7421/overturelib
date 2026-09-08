// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.subsystems.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.overture.lib.sensors.CanCoderConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import org.junit.jupiter.api.Test;

/**
 * The C++ passed and stored every config struct by value, so re-stamping one config between
 * constructions produced independent devices. Java hands out references, so the holders snapshot.
 * These tests pin that, because losing it corrupts odometry silently rather than failing loudly.
 */
class ConfigValueSemanticsTest {
  private static final double kEpsilon = 1e-12;

  /** The classic four-module pattern: build one config, re-stamp it between modules. */
  @Test
  void swerveModuleConfigCopyIsIndependent() {
    SwerveModuleConfig original =
        new SwerveModuleConfig(new SimpleMotorFeedforward(0, 2.0, 0.1), 6, 5);
    original.WheelDiameter = 0.1016;
    original.DriveGearRatio = 7.03;
    original.ModuleName = "Front Left";
    original.EncoderConfig.CanCoderId = 10;
    original.TurnMotorConfig.Slot0.withKP(40);
    original.useFOCDrive = true;
    original.useFOCTurn = true;

    SwerveModuleConfig copy = new SwerveModuleConfig(original);

    // Re-stamp the original the way robot code does for the next corner.
    original.WheelDiameter = 0.2;
    original.DriveGearRatio = 1.0;
    original.ModuleName = "Back Right";
    original.driveMotorId = 4;
    original.turnMotorId = 3;
    original.EncoderConfig.CanCoderId = 12;
    original.TurnMotorConfig.Slot0.withKP(12);
    original.useFOCDrive = false;
    original.useFOCTurn = false;

    assertEquals(0.1016, copy.WheelDiameter, kEpsilon);
    assertEquals(7.03, copy.DriveGearRatio, kEpsilon);
    assertEquals("Front Left", copy.ModuleName);
    assertEquals(6, copy.driveMotorId);
    assertEquals(5, copy.turnMotorId);
    assertEquals(10, copy.EncoderConfig.CanCoderId);
    assertEquals(40.0, copy.TurnMotorConfig.Slot0.kP, kEpsilon);

    // Dropped in an earlier pass of this refactor, which silently ran every module non-FOC.
    assertTrue(copy.useFOCDrive);
    assertTrue(copy.useFOCTurn);

    // The nested objects must be distinct instances, not shared references.
    assertNotSame(original.DriveMotorConfig, copy.DriveMotorConfig);
    assertNotSame(original.TurnMotorConfig, copy.TurnMotorConfig);
    assertNotSame(original.EncoderConfig, copy.EncoderConfig);
    assertNotSame(original.TurnMotorConfig.Slot0, copy.TurnMotorConfig.Slot0);
    assertNotSame(original.FeedForward, copy.FeedForward);
  }

  /**
   * OverTalonFX and SwerveModuleConfig both snapshot a TalonFXConfiguration with {@code clone()}
   * rather than copying it field by field. That is only safe while the clone reaches the nested
   * config objects too; a shallow one would leave every motor built from a re-stamped config
   * sharing one set of gains.
   */
  @Test
  void talonFXConfigurationCloneIsDeep() {
    TalonFXConfiguration original = new TalonFXConfiguration();
    original.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    original.Slot0.withKP(40).withKS(0.15);
    original.CurrentLimits.withStatorCurrentLimit(30.0);

    TalonFXConfiguration copy = original.clone();

    original.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
    original.Slot0.withKP(12).withKS(0.0);
    original.CurrentLimits.withStatorCurrentLimit(60.0);

    assertEquals(NeutralModeValue.Brake, copy.MotorOutput.NeutralMode);
    assertEquals(40.0, copy.Slot0.kP, kEpsilon);
    assertEquals(0.15, copy.Slot0.kS, kEpsilon);
    assertEquals(30.0, copy.CurrentLimits.StatorCurrentLimit, kEpsilon);

    assertNotSame(original.MotorOutput, copy.MotorOutput);
    assertNotSame(original.Slot0, copy.Slot0);
    assertNotSame(original.CurrentLimits, copy.CurrentLimits);
  }

  @Test
  void canCoderConfigCopyIsIndependent() {
    CanCoderConfig original = new CanCoderConfig();
    original.CanCoderId = 9;
    original.Offset = 0.464111328125;

    CanCoderConfig copy = new CanCoderConfig(original);

    original.CanCoderId = 12;
    original.Offset = 0.352294921875;

    assertEquals(9, copy.CanCoderId);
    assertEquals(0.464111328125, copy.Offset, kEpsilon);
  }

  /** The feedforward gains must survive the copy, not reset to zero. */
  @Test
  void feedForwardGainsSurviveTheCopy() {
    SwerveModuleConfig original =
        new SwerveModuleConfig(new SimpleMotorFeedforward(0.1, 2.0879, 0.098433), 1, 2);
    SwerveModuleConfig copy = new SwerveModuleConfig(original);

    assertEquals(0.1, copy.FeedForward.getKs(), kEpsilon);
    assertEquals(2.0879, copy.FeedForward.getKv(), kEpsilon);
    assertEquals(0.098433, copy.FeedForward.getKa(), kEpsilon);
  }
}
