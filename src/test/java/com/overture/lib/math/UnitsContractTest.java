// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.math;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.overture.lib.motorcontrollers.ControllerNeutralMode;
import com.overture.lib.subsystems.swerve.SwerveModuleConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import org.junit.jupiter.api.Test;

/**
 * Pins down the unit contracts the port relies on when talking to Phoenix 6 and WPILib.
 *
 * <p>The C++ original used a units library that converted implicitly at every call boundary. The
 * Java port passes raw doubles, so these conversions are load bearing: getting one wrong compiles
 * cleanly and misbehaves only on the robot. Each test below documents one such boundary.
 */
class UnitsContractTest {
  private static final double kEpsilon = 1e-9;

  /**
   * SwerveModule feeds the turn setpoint to PositionVoltage.withPosition(double), which Phoenix 6
   * documents as rotations. The C++ passed angle.Degrees() and let the units library convert. Using
   * getDegrees() here instead of getRotations() would be a silent 360x error.
   */
  @Test
  void rotation2dToPhoenixRotations() {
    assertEquals(0.125, Rotation2d.fromDegrees(45).getRotations(), kEpsilon);
    assertEquals(0.25, Rotation2d.fromDegrees(90).getRotations(), kEpsilon);
    assertEquals(1.0, Rotation2d.fromDegrees(360).getRotations(), kEpsilon);
    assertEquals(-0.5, Rotation2d.fromDegrees(-180).getRotations(), kEpsilon);
  }

  /**
   * SwerveModule reads the CANcoder absolute position, documented as rotations, and builds a
   * Rotation2d from it. The C++ assigned it to a units::degree_t, which converted turns to degrees
   * on the way in.
   */
  @Test
  void phoenixRotationsToRotation2d() {
    assertEquals(90.0, Rotation2d.fromRotations(0.25).getDegrees(), 1e-12);
    assertEquals(Math.PI, Rotation2d.fromRotations(0.5).getRadians(), 1e-12);
  }

  /**
   * Pins the swerve module defaults carried over from the C++ ModuleConfig.h. The wheel diameter in
   * particular is the factor SwerveModule multiplies drive rotations by, so a regression here
   * silently rescales odometry.
   *
   * <p>The conversion itself lives in SwerveModule.periodic(), which needs real devices and so is
   * not reachable from a plain unit test. This covers the inputs to it, not the arithmetic.
   */
  @Test
  void swerveModuleConfigDefaults() {
    SwerveModuleConfig config = new SwerveModuleConfig(new SimpleMotorFeedforward(0, 0, 0));

    // 4 inch wheel, expressed in meters.
    assertEquals(0.1016, config.WheelDiameter, 1e-12);
    assertEquals(1.0, config.DriveGearRatio, 1e-12);
    assertEquals(1.0, config.TurnGearRatio, 1e-12);

    // Drive brakes and is current limited harder than turn; turn coasts.
    assertEquals(ControllerNeutralMode.Brake, config.DriveMotorConfig.NeutralMode);
    assertEquals(ControllerNeutralMode.Coast, config.TurnMotorConfig.NeutralMode);
    assertEquals(40.0, config.DriveMotorConfig.CurrentLimit, 1e-12);
    assertEquals(120.0, config.DriveMotorConfig.StatorCurrentLimit, 1e-12);
    assertEquals(80.0, config.TurnMotorConfig.StatorCurrentLimit, 1e-12);
    assertEquals(0.25, config.DriveMotorConfig.OpenLoopRampRate, 1e-12);
  }

  /**
   * OverTalonFX clones the gains it is handed, because the Java withSlot0 stores the reference
   * where the C++ member was held by value. Two modules built from one config must not share gains.
   */
  @Test
  void moduleConfigsDoNotShareGainObjects() {
    SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0, 0, 0);
    SwerveModuleConfig a = new SwerveModuleConfig(ff);
    SwerveModuleConfig b = new SwerveModuleConfig(ff);

    a.TurnMotorConfig.PIDConfigs.withKP(40);
    b.TurnMotorConfig.PIDConfigs.withKP(12);

    assertEquals(40.0, a.TurnMotorConfig.PIDConfigs.kP, 1e-12);
    assertEquals(12.0, b.TurnMotorConfig.PIDConfigs.kP, 1e-12);
  }

  /**
   * HeadingSpeedsHelper drives a radian-based ProfiledPIDController. The C++ called
   * SetTolerance(1_deg) on a units::radian controller, so the Java equivalent must convert.
   */
  @Test
  void headingToleranceIsRadians() {
    assertEquals(0.017453292519943295, Units.degreesToRadians(1), kEpsilon);
    assertEquals(Math.PI, Units.degreesToRadians(180), kEpsilon);
  }

  /**
   * AprilTags converts the Limelight camera transform rotation, which WPILib reports in radians,
   * into the degrees that LimelightHelpers expects.
   */
  @Test
  void rotation3dComponentsAreRadians() {
    assertEquals(90.0, Units.radiansToDegrees(Math.PI / 2), 1e-12);
    assertEquals(-45.0, Units.radiansToDegrees(-Math.PI / 4), 1e-12);
  }
}
