// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Arrays;

/**
 * Wiring shared by every swerve drivetrain: the pose estimator and the batched sensor reads.
 *
 * <p>Path following is deliberately not wired here. Whether a robot follows paths with PathPlanner,
 * BLine or something else is the robot project's call, and all any of them need is already public:
 * {@link #getEstimatedPose()}, {@link #resetPose(Pose2d)}, {@link #getCurrentSpeeds()} and {@link
 * #setTargetSpeeds(ChassisSpeeds)}. Worked examples of both live in documentation/swerve.
 *
 * <p>Nor is the drivetrain simulated here. A simulator that drives the devices themselves, such as
 * WPI Emulator, runs the unmodified robot code; a second simulation in here would fight it over the
 * same motor and gyro states.
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

  /**
   * The sensors of all four modules, refreshed once a loop: one group when the modules share a CAN
   * bus, one per module when they do not, because refreshAll refreshes nothing at all when handed
   * signals from two buses.
   */
  private BaseStatusSignal[][] moduleSignalGroups = new BaseStatusSignal[0][];

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
   * Returns the gyro heading.
   *
   * @return the heading
   */
  public abstract Rotation2d getRotation2d();

  /**
   * Builds the pose estimator and takes over reading the modules' sensors. Call this from the
   * constructor of the concrete drivetrain, once its modules exist.
   */
  protected void configureSwerveBase() {
    if (!configuredChassis) {
      SwerveModule[] modules = {
        getFrontLeftModule(), getFrontRightModule(), getBackLeftModule(), getBackRightModule()
      };
      moduleSignalGroups = new BaseStatusSignal[modules.length][];
      boolean oneBus = true;
      for (int i = 0; i < modules.length; i++) {
        moduleSignalGroups[i] = modules[i].handInputsToChassis();
        oneBus &= modules[i].getCanBus().equals(modules[0].getCanBus());
      }
      if (oneBus) {
        moduleSignalGroups =
            new BaseStatusSignal[][] {
              Arrays.stream(moduleSignalGroups)
                  .flatMap(Arrays::stream)
                  .toArray(BaseStatusSignal[]::new)
            };
      }
    }
    configuredChassis = true;

    odometry =
        new SwerveDrivePoseEstimator(
            getKinematics(), new Rotation2d(), modulesPositions, new Pose2d());
  }

  /**
   * Refreshes the sensors of all four modules and reads them into their cached states and
   * positions.
   *
   * <p>With every module on one bus, a single non-blocking native call covers them all, where
   * reading each signal on its own took one call per signal. Modules split across buses take one
   * call each.
   */
  protected void updateModuleInputs() {
    for (BaseStatusSignal[] group : moduleSignalGroups) {
      BaseStatusSignal.refreshAll(group);
    }

    getFrontLeftModule().readInputs();
    getFrontRightModule().readInputs();
    getBackLeftModule().readInputs();
    getBackRightModule().readInputs();
  }

  /**
   * Places the robot at a known pose. This is the reset to hand a path follower.
   *
   * <p>Today this is {@link #resetOdometry(Pose2d)} by another name. It stays separate because the
   * two calls mean different things: a path follower resetting the pose says "the robot is here
   * now", while heading resets and vision corrections also go through resetOdometry. Anything that
   * has to happen on a placement but not on a correction belongs here.
   *
   * @param pose where the robot is
   */
  public void resetPose(Pose2d pose) {
    resetOdometry(pose);
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
}
