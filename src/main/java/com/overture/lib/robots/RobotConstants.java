// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.robots;

/** Timing constants shared across the library. */
public final class RobotConstants {
  private RobotConstants() {}

  /**
   * Control loop period, in seconds.
   *
   * <p>Matches {@link edu.wpi.first.wpilibj.TimedRobot}'s default and the period {@link
   * com.overture.lib.robots.OverRobot} is built with, so the library has one answer rather than
   * three. The C++ used 10 ms here while other parts of the library assumed 20 ms, which is how
   * chassis accelerations ended up reported at half their true size.
   *
   * <p>Anywhere the real period matters the library measures it rather than trusting this, so a
   * robot that schedules its scheduler differently is still correct. This is the fallback.
   */
  public static final double kLoopTime = 0.02;

  /** Timing offset applied to scheduled work, in seconds. */
  public static final double kTimingOffset = 0.005;
}
