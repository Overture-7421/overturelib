// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.robots;

/** Timing constants shared across the library. */
public final class RobotConstants {
  private RobotConstants() {}

  /** Control loop period, in seconds. */
  public static final double kLoopTime = 0.01;

  /** Timing offset applied to scheduled work, in seconds. */
  public static final double kTimingOffset = 0.005;
}
