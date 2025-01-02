package com.overture.lib.subsystems.Swerve.SpeedsHelper;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** The SpeedsHelper interface */
public interface SpeedsHelper {
  /**
   * Alter the input speeds based on the helper's logic
   *
   * @param inputSpeeds The input speeds
   */
  public void alterSpeed(ChassisSpeeds inputSpeeds);

  /** Initialize the helper */
  public void initialize();
}
