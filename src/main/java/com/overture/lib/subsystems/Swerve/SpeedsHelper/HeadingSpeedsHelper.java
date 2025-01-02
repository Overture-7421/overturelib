package com.overture.lib.subsystems.Swerve.SpeedsHelper;

import com.overture.lib.subsystems.Swerve.SwerveChassis;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;

/** The HeadingSpeedsHelper class */
public class HeadingSpeedsHelper implements SpeedsHelper {
  /** The ProfiledPIDController */
  private ProfiledPIDController headingController;
  /** The SwerveChassis */
  private SwerveChassis chassis;
  /** The targetAngle */
  private Angle targetAngle;

  /**
   * The HeadingSpeedsHelper constructor
   *
   * @param headingController - ProfiledPIDController
   * @param chassis - SwerveChassis
   */
  public HeadingSpeedsHelper(ProfiledPIDController headingController, SwerveChassis chassis) {
    this.headingController = headingController;
    this.chassis = chassis;

    this.headingController.enableContinuousInput(-Math.PI, Math.PI);
    this.headingController.setIZone(3);
    this.headingController.setTolerance(0.0174533);
  }

  /**
   * @param targetAngle the targetAngle to set
   */
  public void setTargetAngle(Rotation2d targetAngle) {
    this.targetAngle = targetAngle.getMeasure();
  }

  @Override
  public void alterSpeed(ChassisSpeeds inputSpeeds) {
    double out =
        headingController.calculate(
            chassis.getEstimatedPose().getRotation().getRadians(), targetAngle.magnitude());

    if (headingController.atSetpoint()) {
      out = 0;
    }

    inputSpeeds.omegaRadiansPerSecond = out;
  }

  @Override
  public void initialize() {
    headingController.reset(chassis.getEstimatedPose().getRotation().getRadians());
  }
}
