// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.math;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.Test;

/** Checks the interpolating table and the shoot-on-the-move solver. */
class TargetingWhileMovingTest {
  private static final double kEpsilon = 1e-9;

  private static InterpolatingTable travelTime() {
    // Distance (m) -> projectile travel time (s).
    return new InterpolatingTable(0.0, 0.0, 10.0, 1.0);
  }

  @Test
  void interpolatingTableInterpolatesLinearly() {
    InterpolatingTable table = travelTime();
    assertEquals(0.0, table.get(0.0), kEpsilon);
    assertEquals(0.5, table.get(5.0), kEpsilon);
    assertEquals(1.0, table.get(10.0), kEpsilon);
  }

  @Test
  void interpolatingTableClampsOutsideItsRange() {
    InterpolatingTable table = travelTime();
    assertEquals(0.0, table.get(-5.0), kEpsilon);
    assertEquals(1.0, table.get(50.0), kEpsilon);
  }

  @Test
  void varargsConstructorRejectsOddArgumentCounts() {
    try {
      new InterpolatingTable(1.0, 2.0, 3.0);
      throw new AssertionError("expected an IllegalArgumentException");
    } catch (IllegalArgumentException expected) {
      // expected
    }
  }

  @Test
  void stationaryRobotAimsAtTheRealTarget() {
    TargetingWhileMoving targeting = new TargetingWhileMoving(travelTime());
    targeting.setTargetLocation(new Translation2d(10, 0));

    Translation2d aim =
        targeting.getMovingTarget(
            new Pose2d(0, 0, Rotation2d.kZero), new ChassisSpeeds(0, 0, 0), new ChassisAccels());

    // With no chassis motion the virtual goal collapses onto the real one.
    assertEquals(10.0, aim.getX(), 1e-6);
    assertEquals(0.0, aim.getY(), 1e-6);
  }

  @Test
  void movingTowardTheTargetPullsTheAimPointBack() {
    TargetingWhileMoving targeting = new TargetingWhileMoving(travelTime());
    targeting.setTargetLocation(new Translation2d(10, 0));

    // Driving at the target in +X: the virtual goal shifts toward us (smaller X),
    // because the projectile inherits our forward velocity.
    Translation2d aim =
        targeting.getMovingTarget(
            new Pose2d(0, 0, Rotation2d.kZero), new ChassisSpeeds(2.0, 0, 0), new ChassisAccels());

    assertTrue(aim.getX() < 10.0, "expected the aim point to lead backwards, got " + aim.getX());
    assertEquals(0.0, aim.getY(), 1e-6);
  }

  @Test
  void lateralMotionShiftsTheAimPointSideways() {
    TargetingWhileMoving targeting = new TargetingWhileMoving(travelTime());
    targeting.setTargetLocation(new Translation2d(10, 0));

    Translation2d aim =
        targeting.getMovingTarget(
            new Pose2d(0, 0, Rotation2d.kZero), new ChassisSpeeds(0, 2.0, 0), new ChassisAccels());

    // Strafing +Y means aiming toward -Y to compensate.
    assertTrue(aim.getY() < 0.0, "expected a negative Y offset, got " + aim.getY());
  }

  @Test
  void accelerationCompensationShiftsTheAimPoint() {
    TargetingWhileMoving noAccelComp = new TargetingWhileMoving(travelTime(), 0.0);
    TargetingWhileMoving withAccelComp = new TargetingWhileMoving(travelTime(), 0.5);

    noAccelComp.setTargetLocation(new Translation2d(10, 0));
    withAccelComp.setTargetLocation(new Translation2d(10, 0));

    Pose2d pose = new Pose2d(0, 0, Rotation2d.kZero);
    ChassisSpeeds speeds = new ChassisSpeeds(1.0, 0, 0);
    ChassisAccels accels = new ChassisAccels(4.0, 0, 0);

    double without = noAccelComp.getMovingTarget(pose, speeds, accels).getX();
    double with = withAccelComp.getMovingTarget(pose, speeds, accels).getX();

    assertTrue(with < without, "accel compensation should pull the aim point further back");
  }
}
