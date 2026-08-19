// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.utils;

import com.overture.lib.subsystems.swerve.SwerveChassis;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;

/** Assorted helpers used across robot code. */
public final class UtilityFunctions {
  private UtilityFunctions() {}

  /**
   * Returns whether the driver station reports we are on the red alliance.
   *
   * @return true if we are red, false if we are blue or the alliance is unknown
   */
  public static boolean isRedAlliance() {
    return DriverStation.getAlliance()
        .map(alliance -> alliance == DriverStation.Alliance.Red)
        .orElse(false);
  }

  /**
   * Returns the distance from the chassis to a target pose.
   *
   * @param chassis the chassis to measure from
   * @param targetPose the pose to measure to
   * @return the distance, in meters
   */
  public static double getDistanceToChassis(SwerveChassis chassis, Pose2d targetPose) {
    return chassis.getEstimatedPose().getTranslation().getDistance(targetPose.getTranslation());
  }
}
