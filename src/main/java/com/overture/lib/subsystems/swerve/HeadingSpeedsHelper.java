// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.subsystems.swerve;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

/** Speeds helper that drives the chassis rotation to a target heading. */
public class HeadingSpeedsHelper implements SpeedsHelper {
  private final ProfiledPIDController headingController;
  private final SwerveChassis chassis;
  private double targetAngleRadians;

  /**
   * Constructs a HeadingSpeedsHelper.
   *
   * @param headingController the profiled controller, in radians
   * @param chassis the chassis whose heading is being controlled
   */
  public HeadingSpeedsHelper(ProfiledPIDController headingController, SwerveChassis chassis) {
    this.headingController = headingController;
    this.chassis = chassis;
    this.headingController.enableContinuousInput(-Math.PI, Math.PI);
    this.headingController.setIZone(3);
    this.headingController.setTolerance(Units.degreesToRadians(1));
  }

  /**
   * Sets the heading the chassis should hold.
   *
   * @param targetAngle the target heading
   */
  public void setTargetAngle(Rotation2d targetAngle) {
    this.targetAngleRadians = targetAngle.getRadians();
  }

  @Override
  public void alterSpeed(ChassisSpeeds inputSpeed) {
    double out =
        headingController.calculate(
            chassis.getEstimatedPose().getRotation().getRadians(), targetAngleRadians);

    if (headingController.atSetpoint()) {
      out = 0;
    }

    inputSpeed.omegaRadiansPerSecond = out;
  }

  @Override
  public void initialize() {
    headingController.reset(chassis.getEstimatedPose().getRotation().getRadians());
  }
}
