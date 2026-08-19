// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.utils;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.geometry.Pose2d;

/**
 * Thin wrapper over Phoenix 6's {@link SignalLogger}. Inheriting from it keeps every signal writer
 * (writeBoolean, writeDouble, ...) available through this class.
 */
public class Logging extends SignalLogger {
  /**
   * Logs a pose as an [x, y, degrees] double array.
   *
   * @param path the hoot signal path
   * @param pose the pose to log
   */
  public static void logPose2d(String path, Pose2d pose) {
    logPose2d(path, pose, 0.0);
  }

  /**
   * Logs a pose as an [x, y, degrees] double array.
   *
   * @param path the hoot signal path
   * @param pose the pose to log
   * @param latencySeconds how old the sample is, in seconds
   */
  public static void logPose2d(String path, Pose2d pose, double latencySeconds) {
    SignalLogger.writeDoubleArray(
        path,
        new double[] {pose.getX(), pose.getY(), pose.getRotation().getDegrees()},
        "",
        latencySeconds);
  }

  /** Starts the signal logger. */
  public static void startLogging() {
    SignalLogger.start();
  }

  /** Stops the signal logger. */
  public static void stopLogging() {
    SignalLogger.stop();
  }
}
