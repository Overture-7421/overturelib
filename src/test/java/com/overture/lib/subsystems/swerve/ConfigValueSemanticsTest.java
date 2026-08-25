// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.subsystems.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotSame;

import com.overture.lib.motorcontrollers.ControllerNeutralMode;
import com.overture.lib.motorcontrollers.OverTalonFXConfig;
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
    SwerveModuleConfig original = new SwerveModuleConfig(new SimpleMotorFeedforward(0, 2.0, 0.1));
    original.WheelDiameter = 0.1016;
    original.DriveGearRatio = 7.03;
    original.ModuleName = "Front Left";
    original.DriveMotorConfig.MotorId = 6;
    original.TurnMotorConfig.MotorId = 5;
    original.EncoderConfig.CanCoderId = 10;
    original.TurnMotorConfig.PIDConfigs.withKP(40);

    SwerveModuleConfig copy = new SwerveModuleConfig(original);

    // Re-stamp the original the way robot code does for the next corner.
    original.WheelDiameter = 0.2;
    original.DriveGearRatio = 1.0;
    original.ModuleName = "Back Right";
    original.DriveMotorConfig.MotorId = 4;
    original.TurnMotorConfig.MotorId = 3;
    original.EncoderConfig.CanCoderId = 12;
    original.TurnMotorConfig.PIDConfigs.withKP(12);

    assertEquals(0.1016, copy.WheelDiameter, kEpsilon);
    assertEquals(7.03, copy.DriveGearRatio, kEpsilon);
    assertEquals("Front Left", copy.ModuleName);
    assertEquals(6, copy.DriveMotorConfig.MotorId);
    assertEquals(5, copy.TurnMotorConfig.MotorId);
    assertEquals(10, copy.EncoderConfig.CanCoderId);
    assertEquals(40.0, copy.TurnMotorConfig.PIDConfigs.kP, kEpsilon);

    // The nested objects must be distinct instances, not shared references.
    assertNotSame(original.DriveMotorConfig, copy.DriveMotorConfig);
    assertNotSame(original.TurnMotorConfig, copy.TurnMotorConfig);
    assertNotSame(original.EncoderConfig, copy.EncoderConfig);
    assertNotSame(original.TurnMotorConfig.PIDConfigs, copy.TurnMotorConfig.PIDConfigs);
    assertNotSame(original.FeedForward, copy.FeedForward);
  }

  @Test
  void overTalonFXConfigCopyIsIndependent() {
    OverTalonFXConfig original = new OverTalonFXConfig();
    original.MotorId = 1;
    original.NeutralMode = ControllerNeutralMode.Brake;
    original.useFOC = true;
    original.CurrentLimit = 30.0;
    original.PIDConfigs.withKP(40).withKS(0.15);

    OverTalonFXConfig copy = new OverTalonFXConfig(original);

    original.MotorId = 2;
    original.NeutralMode = ControllerNeutralMode.Coast;
    original.useFOC = false;
    original.CurrentLimit = 60.0;
    original.PIDConfigs.withKP(12).withKS(0.0);

    assertEquals(1, copy.MotorId);
    assertEquals(ControllerNeutralMode.Brake, copy.NeutralMode);
    assertEquals(true, copy.useFOC);
    assertEquals(30.0, copy.CurrentLimit, kEpsilon);
    assertEquals(40.0, copy.PIDConfigs.kP, kEpsilon);
    assertEquals(0.15, copy.PIDConfigs.kS, kEpsilon);
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
        new SwerveModuleConfig(new SimpleMotorFeedforward(0.1, 2.0879, 0.098433));
    SwerveModuleConfig copy = new SwerveModuleConfig(original);

    assertEquals(0.1, copy.FeedForward.getKs(), kEpsilon);
    assertEquals(2.0879, copy.FeedForward.getKv(), kEpsilon);
    assertEquals(0.098433, copy.FeedForward.getKa(), kEpsilon);
  }
}
