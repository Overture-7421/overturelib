package com.overture.lib.subsystems.Swerve;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.overture.lib.math.ChassisAccels;
import com.overture.lib.subsystems.Swerve.SpeedsHelper.SpeedsHelper;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.Optional;

/** The SwerveChassis class */
public class SwerveChassis extends SubsystemBase {
  private SwerveSetpointGenerator m_setpointGenerator;
  private SwerveSetpoint previousSetpoint;
  private Pose2d latestPose;

  private ChassisSpeeds desiredSpeeds = new ChassisSpeeds();
  private ChassisSpeeds currentSpeeds = new ChassisSpeeds();
  private ChassisSpeeds lastSpeeds = new ChassisSpeeds();
  private ChassisAccels currentAccels = new ChassisAccels();

  private Field2d field2d;

  private SwerveDrivePoseEstimator odometry;
  private SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
  private SwerveModuleState[] moduleStates = new SwerveModuleState[4];

  Optional<SpeedsHelper> speedsHelper;
  boolean acceptingVisionMeasurements = false;
  boolean characterizing = false;

  SwerveConfig config;
  SysIdRoutine m_sysIdRoutine;

  /**
   * Constructor for SwerveChassis
   *
   * @param config - SwerveConfig
   */
  public SwerveChassis(SwerveConfig config) {
    this.config = config;

    // Configure AutoBuilder last
    AutoBuilder.configure(
        this::getEstimatedPose,
        this::resetOdometry,
        this::getCurrentSpeeds,
        (speeds, feedforwards) -> setTargetSpeeds(speeds),
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
        config.pathplannerConfig,
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);

    odometry =
        new SwerveDrivePoseEstimator(
            config.kinematics, new Rotation2d(), modulePositions, new Pose2d());

    SmartDashboard.putData("Field", field2d);

    m_sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                this::sysIdVoltage,
                log -> {
                  log.motor("frontLeft")
                      .voltage(config.swerveModules[0].getDriveVoltage())
                      .linearPosition(
                          Meters.of(config.swerveModules[0].getPosition().distanceMeters))
                      .linearVelocity(
                          MetersPerSecond.of(
                              config.swerveModules[0].getState().speedMetersPerSecond));
                  log.motor("frontRight")
                      .voltage(config.swerveModules[1].getDriveVoltage())
                      .linearPosition(
                          Meters.of(config.swerveModules[1].getPosition().distanceMeters))
                      .linearVelocity(
                          MetersPerSecond.of(
                              config.swerveModules[1].getState().speedMetersPerSecond));
                  log.motor("backRight")
                      .voltage(config.swerveModules[2].getDriveVoltage())
                      .linearPosition(
                          Meters.of(config.swerveModules[2].getPosition().distanceMeters))
                      .linearVelocity(
                          MetersPerSecond.of(
                              config.swerveModules[2].getState().speedMetersPerSecond));
                  log.motor("backLeft")
                      .voltage(config.swerveModules[3].getDriveVoltage())
                      .linearPosition(
                          Meters.of(config.swerveModules[3].getPosition().distanceMeters))
                      .linearVelocity(
                          MetersPerSecond.of(
                              config.swerveModules[3].getState().speedMetersPerSecond));
                },
                this));

    m_setpointGenerator =
        new SwerveSetpointGenerator(config.pathplannerConfig, RotationsPerSecond.of(12.5));

    SwerveModuleState[] currentStates = new SwerveModuleState[4];
    previousSetpoint =
        new SwerveSetpoint(
            currentSpeeds,
            currentStates,
            DriveFeedforwards.zeros(config.pathplannerConfig.numModules));
  }

  /** Disables the speed helper */
  public void disableSpeedHelper() {
    speedsHelper = Optional.empty();
  }

  /**
   * Sets the speed helper
   *
   * @param speedsHelper SpeedsHelper object
   */
  public void enableSpeedHelper(SpeedsHelper speedsHelper) {
    speedsHelper.initialize();
    this.speedsHelper = Optional.of(speedsHelper);
  }

  /**
   * Returns the robot relative speeds
   *
   * @return ChassisSpeeds object
   */
  public ChassisSpeeds getCurrentSpeeds() {
    return currentSpeeds;
  }

  /**
   * Returns the robot accelerations
   *
   * @return ChassisAccels object
   */
  public ChassisAccels getCurrentAccels() {
    return currentAccels;
  }

  /**
   * Returns the robot odometry
   *
   * @return Pose2d object
   */
  public Pose2d getEstimatedPose() {
    return latestPose;
  }

  /**
   * Resets the robot odometry
   *
   * @param initPose Pose2d object
   */
  public void resetOdometry(Pose2d initPose) {
    odometry.resetPosition(config.pigeon.getRotation2d(), modulePositions, initPose);
  }

  /**
   * Updates odometry using vision
   *
   * @param pose Pose2d object
   * @param timestamp double
   */
  public void addVisionMeasurement(Pose2d pose, double timestamp) {
    if (acceptingVisionMeasurements) {
      odometry.addVisionMeasurement(pose, timestamp);
    }
  }

  /**
   * Sets if the robot is accepting vision measurements
   *
   * @param acceptingVisionMeasurements boolean
   */
  public void setAcceptingVisionMeasurements(boolean acceptingVisionMeasurements) {
    this.acceptingVisionMeasurements = acceptingVisionMeasurements;
  }

  /**
   * Sets the odometry to desired angle
   *
   * @param angle Desired angle
   */
  void resetHeading(Rotation2d angle) {
    resetOdometry(new Pose2d(getEstimatedPose().getTranslation(), angle));
  }

  /**
   * Sets the target speeds
   *
   * @param speeds ChassisSpeeds object
   */
  public void setTargetSpeeds(ChassisSpeeds speeds) {
    desiredSpeeds = speeds;
  }

  /**
   * Sets the target states
   *
   * @param desiredStates SwerveModuleState array
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    config.swerveModules[0].setState(desiredStates[0]);
    config.swerveModules[1].setState(desiredStates[1]);
    config.swerveModules[2].setState(desiredStates[2]);
    config.swerveModules[3].setState(desiredStates[3]);
  }

  /** Runs the SysId Quasisstatic command */
  Command SysIdQuadstatic(Direction direction) {
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  characterizing = true;
                  sysIdVoltage(Volts.of(0.0));
                }),
            Commands.waitTime(Seconds.of(0.5)),
            m_sysIdRoutine.quasistatic(direction))
        .finallyDo(
            () -> {
              characterizing = false;
            });
  }

  /**
   * Runs the SysId Dynamic command
   *
   * @param direction Direction object
   */
  Command SysIdDinamic(Direction direction) {
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  characterizing = true;
                  sysIdVoltage(Volts.of(0.0));
                }),
            Commands.waitTime(Seconds.of(0.5)),
            m_sysIdRoutine.dynamic(direction))
        .finallyDo(
            () -> {
              characterizing = false;
            });
  }

  /**
   * Sets the voltage for the SysId command
   *
   * @param voltage Voltage object
   */
  public void sysIdVoltage(Voltage voltage) {
    config.swerveModules[0].setVoltageDrive(voltage);
    config.swerveModules[1].setVoltageDrive(voltage);
    config.swerveModules[2].setVoltageDrive(voltage);
    config.swerveModules[3].setVoltageDrive(voltage);
  }

  /** Updates the robot odometry */
  public void updateOdometry() {
    odometry.update(config.pigeon.getRotation2d(), modulePositions);
    latestPose = odometry.getEstimatedPosition();
    currentSpeeds = config.kinematics.toChassisSpeeds(moduleStates);

    currentAccels = new ChassisAccels(currentSpeeds, lastSpeeds);
    lastSpeeds = currentSpeeds;
  }

  /** Shuffleboard update */
  public void shuffleboardPeriodic() {
    SmartDashboard.putNumber("Odometry/AccelX", currentAccels.ax.magnitude());
    SmartDashboard.putNumber("Odometry/AccelY", currentAccels.ay.magnitude());
    SmartDashboard.putNumber("Odometry/AccelOmega", currentAccels.omega.magnitude());

    SmartDashboard.putNumber("Odometry/SpeedX", currentSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Odometry/SpeedY", currentSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("Odometry/SpeedOmega", currentSpeeds.omegaRadiansPerSecond);

    field2d.setRobotPose(latestPose);

    config.swerveModules[0].shuffleboardPeriodic();
    config.swerveModules[1].shuffleboardPeriodic();
    config.swerveModules[2].shuffleboardPeriodic();
    config.swerveModules[3].shuffleboardPeriodic();
  }

  /** Periodic function */
  @Override
  public void periodic() {
    if (characterizing) {
      return;
    }

    if (speedsHelper.isPresent()) {
      speedsHelper.get().alterSpeed(desiredSpeeds);
    }

    modulePositions[0] = config.swerveModules[0].getPosition();
    modulePositions[1] = config.swerveModules[1].getPosition();
    modulePositions[2] = config.swerveModules[2].getPosition();
    modulePositions[3] = config.swerveModules[3].getPosition();

    moduleStates[0] = config.swerveModules[0].getState();
    moduleStates[1] = config.swerveModules[1].getState();
    moduleStates[2] = config.swerveModules[2].getState();
    moduleStates[3] = config.swerveModules[3].getState();

    previousSetpoint =
        m_setpointGenerator.generateSetpoint(previousSetpoint, desiredSpeeds, Seconds.of(0.02));

    updateOdometry();

    setModuleStates(previousSetpoint.moduleStates());
  }
}
