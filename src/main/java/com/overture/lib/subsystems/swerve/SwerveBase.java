// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.subsystems.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.overture.lib.simulation.swerve.SimSwerveDrivetrain;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

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
   * Returns the gyro heading.
   *
   * @return the heading
   */
  public abstract Rotation2d getRotation2d();

  /** The physics simulation, or null on a real robot or when no simulation config was given. */
  private SimSwerveDrivetrain simDrivetrain;

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
          // PathPlanner resetting the pose means "the robot is here now", so the simulated robot
          // is placed there too. This replaces the /PathPlanner/ResetPose topic the external
          // simulator listened on, and with it the trap that republishing an identical pose
          // silently did nothing, because NetworkTables only notified on change.
          //
          // Deliberately not done inside resetOdometry itself, tempting as that is. resetHeading
          // routes through it, and the MegaTag1 watchdog routes through resetHeading, so teleport
          // there would let a vision correction shove the physics robot to match its own estimate
          // and the watchdog could never be caught being wrong.
          resetOdometryPosePublisher.set(pose);
          resetSimulatedPose(pose);
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

    startSimulation();
  }

  /**
   * Describes this drivetrain to the physics simulation, or returns null to run without one.
   *
   * <p>This is the only thing a robot has to write to get a simulated drivetrain, and it lives in
   * the robot project on purpose. Gear ratios and module positions belong next to the {@link
   * SwerveModuleConfig} values they have to agree with: keeping them in a separate simulation
   * project is how a drive ratio of 6.03 sat beside a real robot geared 7.03 for a whole season
   * without anyone noticing.
   *
   * @return the simulation config, or null for no simulation
   */
  protected DriveTrainSimulationConfig getSimulationConfig() {
    return null;
  }

  /**
   * Returns the gyro the simulation should drive, or null to leave it alone.
   *
   * @return the Pigeon to simulate
   */
  protected Pigeon2 getSimulationPigeon() {
    return null;
  }

  /**
   * Returns where the physics says the robot really is, for comparing odometry against.
   *
   * @return the true pose, or empty on a real robot or without a simulation config
   */
  public Optional<Pose2d> getSimulatedPose() {
    return Optional.ofNullable(simDrivetrain).map(SimSwerveDrivetrain::getPose);
  }

  /**
   * Teleports the simulated robot. Does nothing on a real one.
   *
   * @param pose where to place it
   */
  public void resetSimulatedPose(Pose2d pose) {
    if (simDrivetrain != null) {
      simDrivetrain.resetPose(pose);
    }
  }

  // Guarded so the maple-sim classes are never touched on the roboRIO. Java has no preprocessor,
  // so they are linked either way; what matters is that nothing constructs or steps them.
  private void startSimulation() {
    if (!RobotBase.isSimulation() || getSimulationConfig() == null) {
      return;
    }

    simDrivetrain =
        new SimSwerveDrivetrain(getSimulationConfig(), new Pose2d(), getSimulationPigeon());

    // Same order as the module translations and the kinematics.
    getFrontLeftModule().attachSimulation(simDrivetrain.getModule(0));
    getFrontRightModule().attachSimulation(simDrivetrain.getModule(1));
    getBackLeftModule().attachSimulation(simDrivetrain.getModule(2));
    getBackRightModule().attachSimulation(simDrivetrain.getModule(3));
  }

  @Override
  public void simulationPeriodic() {
    if (simDrivetrain != null) {
      simDrivetrain.update();
    }
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
