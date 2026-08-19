// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Wiring shared by every swerve drivetrain: the pose estimator and the PathPlanner AutoBuilder
 * hookup.
 *
 * <p>The C++ version mixes this in alongside SubsystemBase; Java has no multiple inheritance, so
 * this class extends SubsystemBase directly and {@link SwerveChassis} extends it.
 */
public abstract class SwerveBase extends SubsystemBase {
  /** Whether {@link #configureSwerveBase()} has run. */
  protected boolean configuredChassis = false;

  /** Pose estimator fusing odometry and vision. */
  protected SwerveDrivePoseEstimator odometry;

  /** Latest module positions, in front-left, front-right, back-left, back-right order. */
  protected SwerveModulePosition[] modulesPositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  /** Latest module states, in front-left, front-right, back-left, back-right order. */
  protected SwerveModuleState[] modulesStates =
      new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
      };

  private final StructPublisher<Pose2d> resetOdometryPosePublisher =
      NetworkTableInstance.getDefault()
          .getStructTopic("/PathPlanner/ResetPose", Pose2d.struct)
          .publish();

  /**
   * Returns the latest estimated pose.
   *
   * @return the estimated pose
   */
  public abstract Pose2d getEstimatedPose();

  /**
   * Resets the pose estimator to a known pose.
   *
   * @param initPose the pose to reset to
   */
  public abstract void resetOdometry(Pose2d initPose);

  /**
   * Returns the current robot relative speeds.
   *
   * @return the current speeds
   */
  public abstract ChassisSpeeds getCurrentSpeeds();

  /**
   * Requests new robot relative speeds.
   *
   * @param speeds the desired speeds
   */
  public abstract void setTargetSpeeds(ChassisSpeeds speeds);

  /**
   * Returns the maximum speed a single module can reach.
   *
   * @return the maximum module speed, in meters per second
   */
  public abstract double getMaxModuleSpeed();

  /**
   * Returns the distance from the chassis center to a module.
   *
   * @return the drive base radius, in meters
   */
  public abstract double getDriveBaseRadius();

  /**
   * Returns the gyro heading.
   *
   * @return the heading
   */
  public abstract Rotation2d getRotation2d();

  /**
   * Returns the full gyro attitude.
   *
   * @return the attitude
   */
  public abstract Rotation3d getRotation3d();

  /**
   * Wires up PathPlanner and builds the pose estimator. Call this from the constructor of the
   * concrete drivetrain, once its modules exist.
   */
  protected void configureSwerveBase() {
    RobotConfig robotConfig;
    try {
      robotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      throw new RuntimeException("Failed to load the PathPlanner RobotConfig from GUI settings", e);
    }

    configuredChassis = true;
    AutoBuilder.configure(
        this::getEstimatedPose,
        pose -> {
          resetOdometryPosePublisher.set(pose);
          resetOdometry(pose);
        },
        this::getCurrentSpeeds,
        (ChassisSpeeds speeds) -> setTargetSpeeds(speeds),
        new PPHolonomicDriveController(getTranslationPID(), getRotationPID()),
        robotConfig,
        () ->
            DriverStation.getAlliance()
                .map(alliance -> alliance == DriverStation.Alliance.Red)
                .orElse(false),
        this);

    odometry =
        new SwerveDrivePoseEstimator(
            getKinematics(), new Rotation2d(), modulesPositions, new Pose2d());
  }

  /**
   * Returns the front left module.
   *
   * @return the front left module
   */
  protected abstract SwerveModule getFrontLeftModule();

  /**
   * Returns the front right module.
   *
   * @return the front right module
   */
  protected abstract SwerveModule getFrontRightModule();

  /**
   * Returns the back left module.
   *
   * @return the back left module
   */
  protected abstract SwerveModule getBackLeftModule();

  /**
   * Returns the back right module.
   *
   * @return the back right module
   */
  protected abstract SwerveModule getBackRightModule();

  /**
   * Returns the drivetrain kinematics.
   *
   * @return the kinematics
   */
  protected abstract SwerveDriveKinematics getKinematics();

  /**
   * Returns the PathPlanner translation gains.
   *
   * @return the translation PID constants
   */
  protected abstract PIDConstants getTranslationPID();

  /**
   * Returns the PathPlanner rotation gains.
   *
   * @return the rotation PID constants
   */
  protected abstract PIDConstants getRotationPID();
}
