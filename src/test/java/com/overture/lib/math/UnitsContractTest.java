// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.math;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

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
   * SwerveModule converts drive rotations to meters with rotations * diameter * PI. Confirms the
   * factor is circumference, not radius.
   */
  @Test
  void driveRotationsToMeters() {
    double wheelDiameterMeters = 0.1016;
    double circumference = wheelDiameterMeters * Math.PI;

    assertEquals(circumference, 1.0 * wheelDiameterMeters * Math.PI, kEpsilon);
    assertEquals(0.31918, circumference, 1e-5);

    // One rotation of a 4 inch wheel is roughly 0.319 m, not 0.16 m.
    assertTrue(circumference > wheelDiameterMeters);
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
