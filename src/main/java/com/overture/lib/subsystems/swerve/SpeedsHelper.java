// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.subsystems.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Hook that rewrites the chassis speeds on their way to the modules. */
public interface SpeedsHelper {
  /**
   * Alters the requested speeds in place.
   *
   * @param inputSpeed the speeds to modify
   */
  void alterSpeed(ChassisSpeeds inputSpeed);

  /** Called when this helper is enabled on the chassis. */
  default void initialize() {}
}
