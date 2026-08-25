// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.math;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Chassis accelerations, the derivative counterpart of {@link ChassisSpeeds}. */
public class ChassisAccels {
  /** Acceleration along the x axis, in meters per second squared. */
  public double axMetersPerSecondSquared;

  /** Acceleration along the y axis, in meters per second squared. */
  public double ayMetersPerSecondSquared;

  /** Angular acceleration, in radians per second squared. */
  public double omegaRadiansPerSecondSquared;

  /** Accelerations are clamped to this magnitude to reject differentiation noise. */
  private static final double kMaxAccel = 6.0;

  /** Constructs a ChassisAccels with zero acceleration. */
  public ChassisAccels() {
    this(0.0, 0.0, 0.0);
  }

  /**
   * Constructs a ChassisAccels from its components.
   *
   * @param ax acceleration along the x axis, in meters per second squared
   * @param ay acceleration along the y axis, in meters per second squared
   * @param omega angular acceleration, in radians per second squared
   */
  public ChassisAccels(double ax, double ay, double omega) {
    this.axMetersPerSecondSquared = ax;
    this.ayMetersPerSecondSquared = ay;
    this.omegaRadiansPerSecondSquared = omega;
  }

  /**
   * Constructs a ChassisAccels by differentiating two speed samples over the default 20 ms period.
   *
   * @param currentSpeed the current chassis speeds
   * @param lastSpeed the chassis speeds from the previous sample
   */
  public ChassisAccels(ChassisSpeeds currentSpeed, ChassisSpeeds lastSpeed) {
    this(currentSpeed, lastSpeed, 0.02);
  }

  /**
   * Constructs a ChassisAccels by differentiating two speed samples.
   *
   * @param currentSpeed the current chassis speeds
   * @param lastSpeed the chassis speeds from the previous sample
   * @param periodSeconds the time between the two samples, in seconds
   */
  public ChassisAccels(ChassisSpeeds currentSpeed, ChassisSpeeds lastSpeed, double periodSeconds) {
    axMetersPerSecondSquared =
        clamp((currentSpeed.vxMetersPerSecond - lastSpeed.vxMetersPerSecond) / periodSeconds);
    ayMetersPerSecondSquared =
        clamp((currentSpeed.vyMetersPerSecond - lastSpeed.vyMetersPerSecond) / periodSeconds);
    omegaRadiansPerSecondSquared =
        clamp(
            (currentSpeed.omegaRadiansPerSecond - lastSpeed.omegaRadiansPerSecond) / periodSeconds);
  }

  private static double clamp(double accel) {
    if (Math.abs(accel) > kMaxAccel) {
      return Math.copySign(kMaxAccel, accel);
    }
    return accel;
  }

  /**
   * Converts robot relative accelerations into field relative accelerations.
   *
   * @param ax acceleration along the robot's x axis, in meters per second squared
   * @param ay acceleration along the robot's y axis, in meters per second squared
   * @param omega angular acceleration, in radians per second squared
   * @param robotAngle the robot's field relative angle
   * @return the field relative accelerations
   */
  public static ChassisAccels fromRobotRelativeAccels(
      double ax, double ay, double omega, Rotation2d robotAngle) {
    // CCW rotation out of chassis frame
    Translation2d rotated = new Translation2d(ax, ay).rotateBy(robotAngle);
    return new ChassisAccels(rotated.getX(), rotated.getY(), omega);
  }

  /**
   * Converts robot relative accelerations into field relative accelerations.
   *
   * @param robotRelativeAccels the robot relative accelerations
   * @param robotAngle the robot's field relative angle
   * @return the field relative accelerations
   */
  public static ChassisAccels fromRobotRelativeAccels(
      ChassisAccels robotRelativeAccels, Rotation2d robotAngle) {
    return fromRobotRelativeAccels(
        robotRelativeAccels.axMetersPerSecondSquared,
        robotRelativeAccels.ayMetersPerSecondSquared,
        robotRelativeAccels.omegaRadiansPerSecondSquared,
        robotAngle);
  }
}
