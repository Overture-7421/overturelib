// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.subsystems.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Detects wheels losing traction, following the method Orbit 1690 presented in 2024.
 *
 * <p>Take away the part of each module's velocity that comes from the robot rotating, and what is
 * left is that module's opinion of how the robot is translating. All four modules should agree.
 * When they do not, someone is sliding.
 *
 * <p>This only measures. What to do about it is the caller's decision, and the two useful answers
 * are trusting the cameras more for a while and leaving the offending module out of the odometry.
 */
public class SkidDetector {
  /** What the detector saw this loop. */
  public static class Result {
    /**
     * Ratio of the fastest module's translation speed to the slowest. One means perfect agreement.
     */
    public final double ratio;

    /** Whether any module is considered to be skidding. */
    public final boolean skidding;

    /** Per module, whether that module is the one sliding. Indexed like the module states. */
    public final boolean[] skiddingModules;

    /** What the modules that agree think the robot's translation is, in meters per second. */
    public final Translation2d consensusVelocity;

    /**
     * Returns a result meaning "nothing measured yet", for fields that need a value before the
     * first loop.
     *
     * @return an empty result
     */
    public static Result none() {
      return new Result(1.0, false, new boolean[0], Translation2d.kZero);
    }

    Result(double ratio, boolean skidding, boolean[] skiddingModules, Translation2d consensus) {
      this.ratio = ratio;
      this.skidding = skidding;
      this.skiddingModules = skiddingModules;
      this.consensusVelocity = consensus;
    }
  }

  private final double ratioThreshold;
  private final double minimumSpeed;

  /**
   * Constructs a skid detector.
   *
   * @param ratioThreshold how far the fastest module may exceed the slowest before it counts as
   *     skidding, as a ratio. Around 1.3 is a reasonable starting point; too low and every carpet
   *     seam is a skid
   * @param minimumSpeed translation speed below which nothing is reported, in meters per second.
   *     Not in the original method, and necessary: a robot spinning in place or standing still has
   *     almost no translation for the ratio to divide by, so the ratio explodes on noise
   */
  public SkidDetector(double ratioThreshold, double minimumSpeed) {
    this.ratioThreshold = ratioThreshold;
    this.minimumSpeed = minimumSpeed;
  }

  /**
   * Looks for a module that is sliding.
   *
   * @param states the measured module states
   * @param moduleLocations where each module sits relative to the robot centre, in the same order.
   *     {@code SwerveDriveKinematics.getModules()} returns exactly this
   * @param omegaRadiansPerSecond how fast the robot is turning. Prefer the gyro: it is the one
   *     sensor a sliding wheel cannot lie to
   * @return what was seen
   */
  public Result detect(
      SwerveModuleState[] states, Translation2d[] moduleLocations, double omegaRadiansPerSecond) {
    int count = Math.min(states.length, moduleLocations.length);
    if (count == 0) {
      return new Result(1.0, false, new boolean[0], Translation2d.kZero);
    }

    Translation2d[] translationParts = new Translation2d[count];
    double fastest = 0.0;
    double slowest = Double.MAX_VALUE;

    for (int i = 0; i < count; i++) {
      // The module's velocity as a vector, then minus the velocity it would have if the robot were
      // only spinning. Rotating a position by 90 degrees and scaling by omega gives that: a module
      // to the left of centre is pushed forwards when the robot turns counter-clockwise.
      Translation2d measured = new Translation2d(states[i].speedMetersPerSecond, states[i].angle);
      Translation2d fromRotation =
          new Translation2d(
              -omegaRadiansPerSecond * moduleLocations[i].getY(),
              omegaRadiansPerSecond * moduleLocations[i].getX());

      translationParts[i] = measured.minus(fromRotation);

      double speed = translationParts[i].getNorm();
      fastest = Math.max(fastest, speed);
      slowest = Math.min(slowest, speed);
    }

    boolean[] skiddingModules = new boolean[count];

    // Standing still, or turning on the spot, leaves nothing to compare. Bail before dividing.
    if (fastest < minimumSpeed) {
      return new Result(1.0, false, skiddingModules, average(translationParts, skiddingModules));
    }

    double ratio = slowest > 1e-9 ? fastest / slowest : Double.POSITIVE_INFINITY;
    boolean skidding = ratio > ratioThreshold;

    if (skidding) {
      // A wheel that has lost grip spins faster than the robot is really moving, never slower, so
      // the modules reading high are the ones to disbelieve. That asymmetry is the whole reason
      // this is worth doing: it says which module to drop, not just that one of them is wrong.
      double cutoff = slowest * ratioThreshold;
      for (int i = 0; i < count; i++) {
        skiddingModules[i] = translationParts[i].getNorm() > cutoff;
      }
    }

    return new Result(ratio, skidding, skiddingModules, average(translationParts, skiddingModules));
  }

  /** Averages the modules that are not flagged, falling back to all of them if every one is. */
  private static Translation2d average(Translation2d[] parts, boolean[] excluded) {
    Translation2d sum = Translation2d.kZero;
    int used = 0;

    for (int i = 0; i < parts.length; i++) {
      if (!excluded[i]) {
        sum = sum.plus(parts[i]);
        used++;
      }
    }

    if (used == 0) {
      for (Translation2d part : parts) {
        sum = sum.plus(part);
      }
      used = parts.length;
    }

    return used == 0 ? Translation2d.kZero : sum.div(used);
  }
}
