// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.math;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.Test;

/** Checks the differentiation, clamping and frame conversion ported from ChassisAccels.h. */
class ChassisAccelsTest {
  private static final double kEpsilon = 1e-9;

  @Test
  void defaultsToZero() {
    ChassisAccels accels = new ChassisAccels();
    assertEquals(0.0, accels.axMetersPerSecondSquared, kEpsilon);
    assertEquals(0.0, accels.ayMetersPerSecondSquared, kEpsilon);
    assertEquals(0.0, accels.omegaRadiansPerSecondSquared, kEpsilon);
  }

  @Test
  void differentiatesOverTheDefaultPeriod() {
    // 0.02 m/s gained over the default 20 ms period is 1 m/s^2.
    ChassisAccels accels =
        new ChassisAccels(new ChassisSpeeds(0.02, 0.04, 0.06), new ChassisSpeeds(0, 0, 0));

    assertEquals(1.0, accels.axMetersPerSecondSquared, kEpsilon);
    assertEquals(2.0, accels.ayMetersPerSecondSquared, kEpsilon);
    assertEquals(3.0, accels.omegaRadiansPerSecondSquared, kEpsilon);
  }

  @Test
  void differentiatesOverAnExplicitPeriod() {
    ChassisAccels accels =
        new ChassisAccels(new ChassisSpeeds(1.0, 0, 0), new ChassisSpeeds(0, 0, 0), 0.5);

    assertEquals(2.0, accels.axMetersPerSecondSquared, kEpsilon);
  }

  @Test
  void clampsMagnitudeToSixAndKeepsSign() {
    // 1 m/s in 20 ms would be 50 m/s^2; the C++ clamped to +/-6.
    ChassisAccels forward =
        new ChassisAccels(new ChassisSpeeds(1.0, 1.0, 1.0), new ChassisSpeeds(0, 0, 0));
    assertEquals(6.0, forward.axMetersPerSecondSquared, kEpsilon);
    assertEquals(6.0, forward.ayMetersPerSecondSquared, kEpsilon);
    assertEquals(6.0, forward.omegaRadiansPerSecondSquared, kEpsilon);

    ChassisAccels reverse =
        new ChassisAccels(new ChassisSpeeds(-1.0, -1.0, -1.0), new ChassisSpeeds(0, 0, 0));
    assertEquals(-6.0, reverse.axMetersPerSecondSquared, kEpsilon);
    assertEquals(-6.0, reverse.ayMetersPerSecondSquared, kEpsilon);
    assertEquals(-6.0, reverse.omegaRadiansPerSecondSquared, kEpsilon);
  }

  @Test
  void doesNotClampBelowTheLimit() {
    ChassisAccels accels =
        new ChassisAccels(new ChassisSpeeds(0.1, 0, 0), new ChassisSpeeds(0, 0, 0));
    assertEquals(5.0, accels.axMetersPerSecondSquared, kEpsilon);
  }

  @Test
  void rotatesRobotRelativeAccelsIntoTheFieldFrame() {
    // Pointing 90 degrees CCW, a robot-forward accel becomes field +Y.
    ChassisAccels field =
        ChassisAccels.fromRobotRelativeAccels(1.0, 0.0, 2.0, Rotation2d.fromDegrees(90));

    assertEquals(0.0, field.axMetersPerSecondSquared, 1e-12);
    assertEquals(1.0, field.ayMetersPerSecondSquared, 1e-12);
    // Angular acceleration is frame independent.
    assertEquals(2.0, field.omegaRadiansPerSecondSquared, kEpsilon);
  }

  @Test
  void rotationOverloadMatchesTheComponentOverload() {
    ChassisAccels robotRelative = new ChassisAccels(1.0, 2.0, 3.0);
    ChassisAccels viaObject =
        ChassisAccels.fromRobotRelativeAccels(robotRelative, Rotation2d.fromDegrees(37));
    ChassisAccels viaComponents =
        ChassisAccels.fromRobotRelativeAccels(1.0, 2.0, 3.0, Rotation2d.fromDegrees(37));

    assertEquals(
        viaComponents.axMetersPerSecondSquared, viaObject.axMetersPerSecondSquared, kEpsilon);
    assertEquals(
        viaComponents.ayMetersPerSecondSquared, viaObject.ayMetersPerSecondSquared, kEpsilon);
    assertEquals(
        viaComponents.omegaRadiansPerSecondSquared,
        viaObject.omegaRadiansPerSecondSquared,
        kEpsilon);
  }
}
