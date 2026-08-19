// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Computes a virtual target that compensates for the robot's own motion, so a shot taken while
 * driving still converges on the real target.
 */
public class TargetingWhileMoving {
  private Translation2d targetLocation = new Translation2d();
  private final InterpolatingTable distanceToTravelTime;
  private final double accelCompFactor;

  /**
   * Constructs a TargetingWhileMoving with the default acceleration compensation factor.
   *
   * @param distanceToTravelTime maps distance in meters to projectile travel time in seconds
   */
  public TargetingWhileMoving(InterpolatingTable distanceToTravelTime) {
    this(distanceToTravelTime, 0.01);
  }

  /**
   * Constructs a TargetingWhileMoving.
   *
   * @param distanceToTravelTime maps distance in meters to projectile travel time in seconds
   * @param accelCompFactor how far ahead, in seconds, acceleration is extrapolated
   */
  public TargetingWhileMoving(InterpolatingTable distanceToTravelTime, double accelCompFactor) {
    this.distanceToTravelTime = distanceToTravelTime;
    this.accelCompFactor = accelCompFactor;
  }

  /**
   * Sets the real (stationary) target location.
   *
   * @param targetLocation the field relative target location
   */
  public void setTargetLocation(Translation2d targetLocation) {
    this.targetLocation = targetLocation;
  }

  /**
   * Returns the virtual target to aim at while moving.
   *
   * @param robotPose the current robot pose
   * @param fieldRelativeSpeed the field relative chassis speeds
   * @param fieldRelativeAccel the field relative chassis accelerations
   * @return the field relative virtual target location
   */
  public Translation2d getMovingTarget(
      Pose2d robotPose, ChassisSpeeds fieldRelativeSpeed, ChassisAccels fieldRelativeAccel) {
    Translation2d robotLocation = robotPose.getTranslation();
    double distanceToTarget = targetLocation.getDistance(robotLocation);
    double shotTime = distanceToTravelTime.get(distanceToTarget);

    Translation2d movingGoalLocation = new Translation2d();
    Translation2d testGoalLocation;

    for (int i = 0; i < 5; i++) {
      double virtualGoalX =
          targetLocation.getX()
              - shotTime
                  * (fieldRelativeSpeed.vxMetersPerSecond
                      + fieldRelativeAccel.axMetersPerSecondSquared * accelCompFactor);
      double virtualGoalY =
          targetLocation.getY()
              - shotTime
                  * (fieldRelativeSpeed.vyMetersPerSecond
                      + fieldRelativeAccel.ayMetersPerSecondSquared * accelCompFactor);
      testGoalLocation = new Translation2d(virtualGoalX, virtualGoalY);

      double distanceToTestGoal = testGoalLocation.getDistance(robotLocation);
      double newShotTime = distanceToTravelTime.get(distanceToTestGoal);

      if (Math.abs(newShotTime - shotTime) <= 0.010) {
        i = 4;
      }

      if (i == 4) {
        movingGoalLocation = testGoalLocation;
      } else {
        shotTime = newShotTime;
      }
    }

    return movingGoalLocation;
  }
}
