package com.overture.lib.subsystems.Swerve;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.overture.lib.sensors.OverPigeon;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.LinearVelocity;

/** The SwerveConfig class */
public class SwerveConfig {

  /** The SwerveModule[] Note: Starting from front left and going clockwise */
  public SwerveModule[] swerveModules;

  /** The SwerveDriveKinematics */
  public SwerveDriveKinematics kinematics;

  /** The SlewRateLimiter for the drive in X direction */
  public SlewRateLimiter vXLimiter;

  /** The SlewRateLimiter for the drive in Y direction */
  public SlewRateLimiter vYLimiter;

  /** The SlewRateLimiter for the drive rotation */
  public SlewRateLimiter vRotLimiter;

  /** The maxModuleSpeed */
  public LinearVelocity maxModuleSpeed;

  /** The pigeon */
  public OverPigeon pigeon;

  /** The RobotConfig */
  public RobotConfig pathplannerConfig;

  /** The SwerveConfig Constructor */
  public SwerveConfig() {
    try {
      pathplannerConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    maxModuleSpeed = MetersPerSecond.of(pathplannerConfig.moduleConfig.maxDriveVelocityMPS);
  }

  /**
   * @param swerveModules the swerveModules to set
   * @return the SwerveConfig
   */
  public SwerveConfig withSwerveModules(SwerveModule[] swerveModules) {
    this.swerveModules = swerveModules;
    return this;
  }

  /**
   * @param kinematics the kinematics to set
   * @return the SwerveConfig
   */
  public SwerveConfig withKinematics(SwerveDriveKinematics kinematics) {
    this.kinematics = kinematics;
    return this;
  }

  /**
   * @param vXLimiter the vXLimiter to set
   * @return the SwerveConfig
   */
  public SwerveConfig withVXLimiter(SlewRateLimiter vXLimiter) {
    this.vXLimiter = vXLimiter;
    return this;
  }

  /**
   * @param vYLimiter the vYLimiter to set
   * @return the SwerveConfig
   */
  public SwerveConfig withVYLimiter(SlewRateLimiter vYLimiter) {
    this.vYLimiter = vYLimiter;
    return this;
  }

  /**
   * @param vRotLimiter the vRotLimiter to set
   * @return the SwerveConfig
   */
  public SwerveConfig withVRotLimiter(SlewRateLimiter vRotLimiter) {
    this.vRotLimiter = vRotLimiter;
    return this;
  }

  /**
   * Note: This variable is set from the RobotConfig, should not be set manually
   *
   * @param maxModuleSpeed the maxModuleSpeed to set
   * @return the SwerveConfig
   */
  public SwerveConfig withMaxModuleSpeed(LinearVelocity maxModuleSpeed) {
    this.maxModuleSpeed = maxModuleSpeed;
    return this;
  }

  /**
   * @param pigeonId the pigeon to set
   * @return the SwerveConfig
   */
  public SwerveConfig withPigeon(int pigeonId) {
    this.pigeon = new OverPigeon(pigeonId, "OverCANivore");
    return this;
  }
}
