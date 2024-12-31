package com.overture.lib.math;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

/**
 * The TargetingWhileMoving class
 *
 * <p>TargetingWhileMoving is a class that contains the logic for targeting while moving.
 */
public class TargetingWhileMoving {
  private Translation2d targetLocation;
  private Time accelCompFactor;
  private InterpolatingTreeMap<Distance, Time> distanceToTravelTime;

  /**
   * Constructor for TargetingWhileMoving, initializes the TargetingWhileMoving with the given
   * InterpolatingTreeMap and Time.
   *
   * @param distanceToTravelTime The InterpolatingTreeMap of Distance to Time.
   * @param accelCompFactor The Time to compensate for acceleration in seconds.
   */
  public TargetingWhileMoving(
      InterpolatingTreeMap<Distance, Time> distanceToTravelTime, Time accelCompFactor) {
    this.distanceToTravelTime = distanceToTravelTime;
    this.accelCompFactor = accelCompFactor;
  }

  /**
   * Constructor for TargetingWhileMoving, initializes the TargetingWhileMoving with the given
   * InterpolatingTreeMap.
   *
   * @param distanceToTravelTime The InterpolatingTreeMap of Distance to Time.
   */
  public TargetingWhileMoving(InterpolatingTreeMap<Distance, Time> distanceToTravelTime) {
    this(distanceToTravelTime, Second.of(0.01));
  }

  /**
   * Sets the target location
   *
   * @param targetLocation The target location
   */
  public void setTargetLocation(Translation2d targetLocation) {
    this.targetLocation = targetLocation;
  }

  /**
   * Gets the moving target
   *
   * @param robotPose The robot pose
   * @param fieldRelativeSpeeds The field relative speeds
   * @param fieldRelativeAccel The field relative acceleration
   * @return The moving target
   */
  public Translation2d getMovingTarget(
      Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds, ChassisAccels fieldRelativeAccel) {
    Translation2d robotLocation = robotPose.getTranslation();
    Distance distanceToTarget = Meters.of(robotLocation.getDistance(targetLocation));
    Time shotTime = distanceToTravelTime.get(distanceToTarget);

    Translation2d movingGoalLocation = targetLocation;
    Translation2d testGoalLocation;
    for (int i = 0; i < 5; i++) {
      Distance virtualGoalX =
          Meters.of(
              targetLocation.getX()
                  - shotTime.magnitude()
                      * (fieldRelativeSpeeds.vxMetersPerSecond
                          + fieldRelativeAccel.ax.magnitude() * accelCompFactor.magnitude()));

      Distance virtualGoalY =
          Meters.of(
              targetLocation.getY()
                  - shotTime.magnitude()
                      * (fieldRelativeSpeeds.vyMetersPerSecond
                          + fieldRelativeAccel.ay.magnitude() * accelCompFactor.magnitude()));
      testGoalLocation = new Translation2d(virtualGoalX, virtualGoalY);
      Distance distanceToTestGoal = Meters.of(testGoalLocation.getDistance(robotLocation));

      Time newShotTime = distanceToTravelTime.get(distanceToTestGoal);

      if (newShotTime.minus(shotTime).abs(Seconds) <= 0.01) {
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
