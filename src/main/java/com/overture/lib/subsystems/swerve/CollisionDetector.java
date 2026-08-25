// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.subsystems.swerve;

/**
 * Flags the robot being hit, following the method Orbit 1690 presented in 2024.
 *
 * <p>A collision is an impulse: it shows up for a loop or two and is gone before anything
 * downstream can react to it. So the trigger is held for a while afterwards, and the useful
 * question is not "am I being hit right now" but "have I been hit recently enough that I should not
 * trust where I think I am".
 *
 * <p>Note on the threshold. The original used the roboRIO's accelerometer and called anything over
 * 2 g a collision. A Pigeon 2 reports acceleration in g but saturates at 2, so that exact number
 * can never be exceeded on this sensor and a detector built on it would sit silent all match. Keep
 * the threshold below the rail. Hitting the rail is itself a hard collision, so nothing is lost
 * except knowing how hard.
 */
public class CollisionDetector {
  private final double thresholdGs;
  private final double holdSeconds;

  private double lastTriggerTimestamp = Double.NEGATIVE_INFINITY;
  private double lastAccelerationGs;

  /**
   * Constructs a collision detector.
   *
   * @param thresholdGs acceleration magnitude that counts as a hit, in g. Keep it under 2 on a
   *     Pigeon 2, and above what the drivetrain can pull on its own: a swerve accelerating hard
   *     manages roughly 1 g, so somewhere around 1.5 separates being hit from driving
   * @param holdSeconds how long the flag stays raised after a hit, in seconds
   */
  public CollisionDetector(double thresholdGs, double holdSeconds) {
    this.thresholdGs = thresholdGs;
    this.holdSeconds = holdSeconds;
  }

  /**
   * Feeds the detector a new reading.
   *
   * @param accelerationGs magnitude of the acceleration in the floor plane, in g
   * @param timestampSeconds the current time, in seconds. Passed in rather than read, so this stays
   *     testable off the robot
   * @return whether the flag is currently raised
   */
  public boolean update(double accelerationGs, double timestampSeconds) {
    lastAccelerationGs = accelerationGs;

    if (accelerationGs >= thresholdGs) {
      lastTriggerTimestamp = timestampSeconds;
    }

    return isColliding(timestampSeconds);
  }

  /**
   * Returns whether a hit is still being held from a recent trigger.
   *
   * @param timestampSeconds the current time, in seconds
   * @return whether the flag is raised
   */
  public boolean isColliding(double timestampSeconds) {
    return timestampSeconds - lastTriggerTimestamp < holdSeconds;
  }

  /**
   * Returns the last acceleration magnitude handed to {@link #update}.
   *
   * @return the acceleration, in g
   */
  public double getLastAccelerationGs() {
    return lastAccelerationGs;
  }
}
