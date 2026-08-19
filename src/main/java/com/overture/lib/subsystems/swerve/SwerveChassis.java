// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.subsystems.swerve;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.overture.lib.math.ChassisAccels;
import com.overture.lib.motorcontrollers.ControllerNeutralMode;
import com.overture.lib.robots.RobotConstants;
import com.overture.lib.utils.Logging;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.Optional;

/**
 * A swerve drivetrain. Concrete robots extend this and supply the modules, kinematics and
 * PathPlanner gains.
 */
public abstract class SwerveChassis extends SwerveBase {
  private Pose2d latestPose = new Pose2d();

  private ChassisSpeeds desiredSpeeds = new ChassisSpeeds();
  private ChassisSpeeds currentSpeeds = new ChassisSpeeds();
  private ChassisSpeeds lastSpeeds = new ChassisSpeeds();
  private ChassisAccels currentAccels = new ChassisAccels();

  private Optional<SpeedsHelper> speedsHelper = Optional.empty();
  private boolean acceptingVisionMeasurements = false;
  private boolean xModeEnabled = false;

  private final StructPublisher<Pose2d> posePublisher =
      NetworkTableInstance.getDefault()
          .getStructTopic("SmartDashboard/SwerveChassis/Odometry/Pose", Pose2d.struct)
          .publish();

  private boolean characterizing = false;

  private final SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              driveVoltage -> sysIdVoltage(driveVoltage.in(Volts)),
              log -> {
                log.motor("frontRight")
                    .voltage(Volts.of(getFrontRightModule().getVoltageDrive()))
                    .linearPosition(Meters.of(getFrontRightModule().getPosition().distanceMeters))
                    .linearVelocity(
                        MetersPerSecond.of(getFrontRightModule().getState().speedMetersPerSecond));
                log.motor("frontLeft")
                    .voltage(Volts.of(getFrontLeftModule().getVoltageDrive()))
                    .linearPosition(Meters.of(getFrontLeftModule().getPosition().distanceMeters))
                    .linearVelocity(
                        MetersPerSecond.of(getFrontLeftModule().getState().speedMetersPerSecond));
                log.motor("backRight")
                    .voltage(Volts.of(getBackRightModule().getVoltageDrive()))
                    .linearPosition(Meters.of(getBackRightModule().getPosition().distanceMeters))
                    .linearVelocity(
                        MetersPerSecond.of(getBackRightModule().getState().speedMetersPerSecond));
                log.motor("backLeft")
                    .voltage(Volts.of(getBackLeftModule().getVoltageDrive()))
                    .linearPosition(Meters.of(getBackLeftModule().getPosition().distanceMeters))
                    .linearVelocity(
                        MetersPerSecond.of(getBackLeftModule().getState().speedMetersPerSecond));
              },
              this));

  /** Builds a swerve chassis. */
  public SwerveChassis() {
    super();
  }

  /** Disables the active speeds helper. */
  public void disableSpeedHelper() {
    speedsHelper = Optional.empty();
  }

  /**
   * Enables a speeds helper, which rewrites the requested speeds every loop.
   *
   * @param speedsHelper the helper to enable
   */
  public void enableSpeedHelper(SpeedsHelper speedsHelper) {
    speedsHelper.initialize();
    this.speedsHelper = Optional.of(speedsHelper);
  }

  @Override
  public ChassisSpeeds getCurrentSpeeds() {
    return currentSpeeds;
  }

  /**
   * Returns the robot accelerations.
   *
   * @return the current accelerations
   */
  public ChassisAccels getCurrentAccels() {
    return currentAccels;
  }

  @Override
  public Pose2d getEstimatedPose() {
    return latestPose;
  }

  @Override
  public void resetOdometry(Pose2d initPose) {
    odometry.resetPosition(getRotation2d(), modulesPositions, initPose);
  }

  /**
   * Feeds a vision pose into the estimator, if vision measurements are being accepted.
   *
   * @param pose the vision estimated pose
   * @param timestamp the timestamp of the sample, in seconds
   * @param stdDevs the standard deviations of the measurement
   */
  public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {
    if (!acceptingVisionMeasurements) {
      return;
    }
    odometry.addVisionMeasurement(pose, timestamp, stdDevs);
  }

  /**
   * Sets whether vision measurements are fused into the estimator.
   *
   * @param acceptingVisionMeasurements whether to accept vision measurements
   */
  public void setAcceptingVisionMeasurements(boolean acceptingVisionMeasurements) {
    this.acceptingVisionMeasurements = acceptingVisionMeasurements;
  }

  /** Sets the odometry heading to zero, keeping the translation. */
  public void resetHeading() {
    resetHeading(0);
  }

  /**
   * Sets the odometry heading, keeping the translation.
   *
   * @param angleDegrees the desired heading, in degrees
   */
  public void resetHeading(double angleDegrees) {
    Pose2d actualOdometry = getEstimatedPose();
    Pose2d newOdometry =
        new Pose2d(
            actualOdometry.getX(), actualOdometry.getY(), Rotation2d.fromDegrees(angleDegrees));
    resetOdometry(newOdometry);
  }

  /**
   * Points the wheels into an X so the robot resists being pushed.
   *
   * @param enabled whether X mode is engaged
   */
  public void setXMode(boolean enabled) {
    if (enabled == xModeEnabled) {
      return;
    }
    xModeEnabled = enabled;

    if (enabled) {
      getFrontLeftModule().setDriveNeutralMode(ControllerNeutralMode.Brake);
      getFrontRightModule().setDriveNeutralMode(ControllerNeutralMode.Brake);
      getBackLeftModule().setDriveNeutralMode(ControllerNeutralMode.Brake);
      getBackRightModule().setDriveNeutralMode(ControllerNeutralMode.Brake);
    } else {
      getFrontLeftModule().restoreDriveNeutralMode();
      getFrontRightModule().restoreDriveNeutralMode();
      getBackLeftModule().restoreDriveNeutralMode();
      getBackRightModule().restoreDriveNeutralMode();
    }
  }

  @Override
  public void setTargetSpeeds(ChassisSpeeds speeds) {
    desiredSpeeds = ChassisSpeeds.discretize(speeds, RobotConstants.kLoopTime);
  }

  private void setModuleStates(SwerveModuleState[] desiredStates) {
    getFrontLeftModule().setState(desiredStates[0]);
    getFrontRightModule().setState(desiredStates[1]);
    getBackLeftModule().setState(desiredStates[2]);
    getBackRightModule().setState(desiredStates[3]);
  }

  /**
   * Returns the SysId quasistatic command.
   *
   * @param direction the direction to characterize in
   * @return the command
   */
  public Command sysIdQuadstatic(SysIdRoutine.Direction direction) {
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  characterizing = true;
                  sysIdVoltage(0);
                }),
            Commands.waitSeconds(0.5),
            sysIdRoutine.quasistatic(direction))
        .finallyDo(() -> characterizing = false);
  }

  /**
   * Returns the SysId dynamic command.
   *
   * @param direction the direction to characterize in
   * @return the command
   */
  public Command sysIdDinamic(SysIdRoutine.Direction direction) {
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  characterizing = true;
                  sysIdVoltage(0);
                }),
            Commands.waitSeconds(0.5),
            sysIdRoutine.dynamic(direction))
        .finallyDo(() -> characterizing = false);
  }

  /**
   * Applies a raw voltage to every drive motor, used by SysId.
   *
   * @param voltage the voltage to apply
   */
  public void sysIdVoltage(double voltage) {
    getFrontLeftModule().setVoltageDrive(voltage);
    getFrontRightModule().setVoltageDrive(voltage);
    getBackRightModule().setVoltageDrive(voltage);
    getBackLeftModule().setVoltageDrive(voltage);
  }

  private void updateOdometry() {
    odometry.update(getRotation2d(), modulesPositions);
    latestPose = odometry.getEstimatedPosition();
    currentSpeeds = getKinematics().toChassisSpeeds(modulesStates);

    currentAccels = new ChassisAccels(currentSpeeds, lastSpeeds);
    lastSpeeds = currentSpeeds;
  }

  /** Publishes chassis telemetry. */
  public void shuffleboardPeriodic() {
    SmartDashboard.putNumber("SwerveChassis/Odometry/LinearX", desiredSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("SwerveChassis/Odometry/LinearY", desiredSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("SwerveChassis/Odometry/Angular", desiredSpeeds.omegaRadiansPerSecond);

    SmartDashboard.putNumber(
        "SwerveChassis/Odometry/AccelX", currentAccels.axMetersPerSecondSquared);
    SmartDashboard.putNumber(
        "SwerveChassis/Odometry/AccelY", currentAccels.ayMetersPerSecondSquared);
    SmartDashboard.putNumber(
        "SwerveChassis/Odometry/AccelOmega", currentAccels.omegaRadiansPerSecondSquared);

    SmartDashboard.putNumber("SwerveChassis/Odometry/SpeedX", desiredSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("SwerveChassis/Odometry/SpeedY", desiredSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber(
        "SwerveChassis/Odometry/SpeedOmega", desiredSpeeds.omegaRadiansPerSecond);

    SmartDashboard.putBoolean(
        "SwerveChassis/Odometry/AcceptingVision", acceptingVisionMeasurements);

    posePublisher.set(latestPose);

    SmartDashboard.putNumberArray(
        "SwerveChassis/Control/DesiredSpeeds",
        new double[] {
          desiredSpeeds.vxMetersPerSecond,
          desiredSpeeds.vyMetersPerSecond,
          desiredSpeeds.omegaRadiansPerSecond
        });

    getFrontLeftModule().shuffleboardPeriodic();
    getFrontRightModule().shuffleboardPeriodic();
    getBackRightModule().shuffleboardPeriodic();
    getBackLeftModule().shuffleboardPeriodic();
  }

  @Override
  public void periodic() {
    if (!configuredChassis) {
      throw new IllegalStateException("Have not called SwerveBase.configureSwerveBase!!!");
    }

    if (characterizing) {
      return;
    }

    speedsHelper.ifPresent(helper -> helper.alterSpeed(desiredSpeeds));

    modulesPositions[0] = getFrontLeftModule().getPosition();
    modulesPositions[1] = getFrontRightModule().getPosition();
    modulesPositions[2] = getBackLeftModule().getPosition();
    modulesPositions[3] = getBackRightModule().getPosition();

    modulesStates[0] = getFrontLeftModule().getState();
    modulesStates[1] = getFrontRightModule().getState();
    modulesStates[2] = getBackLeftModule().getState();
    modulesStates[3] = getBackRightModule().getState();

    SwerveModuleState[] desiredStates = getKinematics().toSwerveModuleStates(desiredSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, getMaxModuleSpeed());

    if (xModeEnabled) {
      desiredStates[0] = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
      desiredStates[1] = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
      desiredStates[2] = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
      desiredStates[3] = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
    }

    updateOdometry();

    Logging.logPose2d("/Swerve/Chassis/Pose", latestPose);

    setModuleStates(desiredStates);
  }
}
