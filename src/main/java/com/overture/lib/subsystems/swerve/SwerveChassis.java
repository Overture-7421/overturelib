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
import edu.wpi.first.wpilibj.Timer;
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
  private double lastPeriodicTimestamp = Timer.getFPGATimestamp();
  private ChassisSpeeds currentSpeeds = new ChassisSpeeds();
  private ChassisSpeeds lastSpeeds = new ChassisSpeeds();
  private ChassisAccels currentAccels = new ChassisAccels();

  // Thresholds are a starting point, not a tuned value: 1.3 means the fastest module may read 30%
  // above the slowest before anyone is accused, and below 0.3 m/s of translation there is not
  // enough motion for the ratio to mean anything.
  private final SkidDetector skidDetector = new SkidDetector(1.3, 0.3);
  private SkidDetector.Result skidResult = SkidDetector.Result.none();

  // 1.5 g is above what a swerve pulls on its own and below the Pigeon 2 rail at 2 g. Held for a
  // quarter second, because a hit is over in a loop or two and nothing downstream samples that
  // fast.
  private final CollisionDetector collisionDetector = new CollisionDetector(1.5, 0.25);

  private Optional<SpeedsHelper> speedsHelper = Optional.empty();
  private boolean acceptingVisionMeasurements = false;
  private boolean xModeEnabled = false;

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

  /**
   * Returns what the skid detector last saw.
   *
   * @return the latest skid result
   */
  public SkidDetector.Result getSkidResult() {
    return skidResult;
  }

  /**
   * Returns how fast the robot is turning, for the skid detector.
   *
   * <p>Defaults to the kinematics answer, which is honest but circular: it is derived from the same
   * four modules the detector is trying to catch lying. Override it with the gyro's angular
   * velocity, the one measurement a sliding wheel cannot corrupt, and the detector gets sharper.
   *
   * @return the angular velocity, in radians per second
   */
  protected double getAngularVelocityRadiansPerSecond() {
    return currentSpeeds.omegaRadiansPerSecond;
  }

  /**
   * Returns how hard the robot is being accelerated in the floor plane, for the collision detector.
   *
   * <p>Zero by default, which leaves collision detection switched off. Override it with the gyro,
   * which is already carrying an accelerometer:
   *
   * <pre>
   * &#64;Override
   * protected double getMeasuredAccelerationGs() {
   *   return Math.hypot(
   *       pigeon.getAccelerationX().getValueAsDouble(),
   *       pigeon.getAccelerationY().getValueAsDouble());
   * }
   * </pre>
   *
   * <p>X and Y rather than all three axes: the Pigeon's reading includes gravity, so Z sits at
   * about 1 g on a robot that is doing nothing at all.
   *
   * @return the acceleration magnitude, in g
   */
  protected double getMeasuredAccelerationGs() {
    return 0.0;
  }

  /**
   * Returns whether the robot has been hit recently.
   *
   * @return whether the collision flag is raised
   */
  public boolean isColliding() {
    return collisionDetector.isColliding(Timer.getFPGATimestamp());
  }

  @Override
  public Pose2d getEstimatedPose() {
    return latestPose;
  }

  @Override
  public void resetOdometry(Pose2d initPose) {
    odometry.resetPosition(getRotation2d(), modulesPositions, initPose);
    // Refresh the cache getEstimatedPose() serves. Without this the estimator knows the new pose
    // but every reader keeps seeing the old one until the next periodic(), and CommandScheduler
    // runs subsystem periodics BEFORE it initializes commands. PathPlanner registers
    // resetOdometry as its reset consumer and getEstimatedPose as its pose supplier, so a
    // reset-then-follow scheduled from a trigger would start the path from the pre-reset pose.
    latestPose = odometry.getEstimatedPosition();
  }

  /**
   * Samples the pose estimator at a past timestamp.
   *
   * @param timestampSeconds the FPGA timestamp to sample at
   * @return the estimated pose at that time, empty if it is outside the estimator's buffer
   */
  public Optional<Pose2d> getEstimatedPoseAt(double timestampSeconds) {
    return odometry.sampleAt(timestampSeconds);
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
    // Copied rather than aliased. A caller usually owns one ChassisSpeeds it refills every loop,
    // and periodic() hands a speeds helper something it may rewrite in place.
    desiredSpeeds =
        new ChassisSpeeds(
            speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
  }

  /**
   * Returns how long it has been since the last periodic call, for discretizing the speeds.
   *
   * <p>Measured rather than taken from {@link RobotConstants#kLoopTime}, because that constant only
   * describes reality if robot code drives the CommandScheduler at exactly that rate. A robot that
   * leaves the scheduler on the default TimedRobot period gets half the compensation it needs, and
   * nothing says so.
   *
   * @return the elapsed time, in seconds
   */
  private double getPeriodicPeriod() {
    double now = Timer.getFPGATimestamp();
    double period = now - lastPeriodicTimestamp;
    lastPeriodicTimestamp = now;

    // First call, and the gap either side of a SysId routine, land outside anything plausible. Fall
    // back to the configured period instead of discretizing by zero, which is a silent no-op.
    return period > 0.0 && period < 0.5 ? period : RobotConstants.kLoopTime;
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

  private void updateOdometry(double periodSeconds) {
    odometry.update(getRotation2d(), modulesPositions);
    latestPose = odometry.getEstimatedPosition();
    currentSpeeds = getKinematics().toChassisSpeeds(modulesStates);

    // Differentiated over the period actually measured. The two argument constructor assumes 20 ms,
    // so on a robot scheduling its loop faster every reported acceleration came out scaled by the
    // ratio -- half, at a 10 ms loop -- and TargetingWhileMoving under-corrected by the same
    // factor.
    currentAccels = new ChassisAccels(currentSpeeds, lastSpeeds, periodSeconds);

    // Measured only, for now: nothing downstream acts on it. Watch /Swerve/Chassis/SkidRatio on a
    // real log first, because the threshold that separates a slipping wheel from a carpet seam is
    // a property of your carpet and your wheels, not something to guess in a library.
    skidResult =
        skidDetector.detect(
            modulesStates, getKinematics().getModules(), getAngularVelocityRadiansPerSecond());

    Logging.logDouble(
        "/Swerve/Chassis/SkidRatio", skidResult.ratio, "", Logging.Destination.LOG_ONLY);
    Logging.logBoolean(
        "/Swerve/Chassis/Skidding", skidResult.skidding, Logging.Destination.LOG_ONLY);
    lastSpeeds = currentSpeeds;
  }

  /**
   * Publishes chassis telemetry, recorded and live.
   *
   * <p>Three separate sets of signals used to carry the same numbers: LinearX/Y/Angular,
   * SpeedX/Y/Omega and the DesiredSpeeds array were all the requested speeds, spelled three ways.
   * They are one ChassisSpeeds struct now, and the measured speeds are published beside them, which
   * is what you actually want when the robot is not going where it was asked to.
   */
  public void shuffleboardPeriodic() {
    Logging.logChassisSpeeds(
        "/Swerve/Chassis/DesiredSpeeds", desiredSpeeds, Logging.Destination.BOTH);
    Logging.logChassisSpeeds(
        "/Swerve/Chassis/CurrentSpeeds", currentSpeeds, Logging.Destination.BOTH);

    Logging.logDouble(
        "/Swerve/Chassis/Accel/X",
        currentAccels.axMetersPerSecondSquared,
        "m/s^2",
        Logging.Destination.BOTH);
    Logging.logDouble(
        "/Swerve/Chassis/Accel/Y",
        currentAccels.ayMetersPerSecondSquared,
        "m/s^2",
        Logging.Destination.BOTH);
    Logging.logDouble(
        "/Swerve/Chassis/Accel/Omega",
        currentAccels.omegaRadiansPerSecondSquared,
        "rad/s^2",
        Logging.Destination.BOTH);

    Logging.logBoolean(
        "/Swerve/Chassis/AcceptingVision", acceptingVisionMeasurements, Logging.Destination.BOTH);

    // The array of measured states is what AdvantageScope's swerve view consumes.
    Logging.logModuleStates("/Swerve/Modules/States", modulesStates, Logging.Destination.BOTH);
    Logging.logModulePositions(
        "/Swerve/Modules/Positions", modulesPositions, Logging.Destination.LOG_ONLY);

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

    // Read the modules here, first, rather than leaving it to their own periodic. They register
    // with the CommandScheduler after this class does -- a subclass's fields are initialised after
    // its super constructor runs, and the scheduler keeps subsystems in insertion order -- so their
    // periodic runs after this one. Everything below was therefore working from module data one
    // full loop old, which is invisible in the odometry error because the wheels are what odometry
    // measures: the pose was self consistent and simply late, and vision fused against it with an
    // honest timestamp and a stale baseline.
    //
    // Above the characterizing check on purpose. SysId logs module position and velocity, so the
    // modules have to be current even on the loops where the rest of this is skipped.
    getFrontLeftModule().updateInputs();
    getFrontRightModule().updateInputs();
    getBackLeftModule().updateInputs();
    getBackRightModule().updateInputs();

    if (characterizing) {
      return;
    }

    // Measured once and shared. Reading it twice would hand the second caller the time since the
    // first read, which is almost nothing.
    double periodSeconds = getPeriodicPeriod();

    // The helper gets a copy, so desiredSpeeds stays the request that came in. It is the signal
    // that gets logged, and on a loop where nothing called setTargetSpeeds a helper would
    // otherwise be reading back its own previous answer.
    ChassisSpeeds targetSpeeds =
        new ChassisSpeeds(
            desiredSpeeds.vxMetersPerSecond,
            desiredSpeeds.vyMetersPerSecond,
            desiredSpeeds.omegaRadiansPerSecond);
    speedsHelper.ifPresent(helper -> helper.alterSpeed(targetSpeeds));

    modulesPositions[0] = getFrontLeftModule().getPosition();
    modulesPositions[1] = getFrontRightModule().getPosition();
    modulesPositions[2] = getBackLeftModule().getPosition();
    modulesPositions[3] = getBackRightModule().getPosition();

    modulesStates[0] = getFrontLeftModule().getState();
    modulesStates[1] = getFrontRightModule().getState();
    modulesStates[2] = getBackLeftModule().getState();
    modulesStates[3] = getBackRightModule().getState();

    // Discretized here, last, rather than when the speeds arrived. discretize counter-rotates vx
    // and vy to pay for the heading change across one period, so it has to see the omega that is
    // actually about to be commanded. A heading helper replaces omega outright, so compensating
    // for the driver's requested omega sent the robot sideways whenever it auto-aimed while
    // translating: the correction was paying for a rotation that was not happening.
    SwerveModuleState[] desiredStates =
        getKinematics().toSwerveModuleStates(ChassisSpeeds.discretize(targetSpeeds, periodSeconds));
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, getMaxModuleSpeed());

    if (xModeEnabled) {
      desiredStates[0] = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
      desiredStates[1] = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
      desiredStates[2] = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
      desiredStates[3] = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
    }

    updateOdometry(periodSeconds);

    Logging.logPose("/Swerve/Chassis/Pose", latestPose, Logging.Destination.BOTH);

    setModuleStates(desiredStates);
  }
}
