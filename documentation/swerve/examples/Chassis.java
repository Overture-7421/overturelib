// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Chassis;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.InvertedValue;
import com.overture.lib.sensors.OverPigeon;
import com.overture.lib.subsystems.swerve.SwerveChassis;
import com.overture.lib.subsystems.swerve.SwerveModule;
import com.overture.lib.subsystems.swerve.SwerveModuleConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

/**
 * Shelby's drivetrain, from FRC-Shelby-2026OS, as an example of a concrete {@link SwerveChassis}.
 *
 * <p>Nothing in here knows how the robot follows paths. A path follower is wired in from outside,
 * through the public methods the chassis inherits: see PathPlannerConfig and BLinePaths next to
 * this file. Swapping one for the other does not touch this class.
 */
public class Chassis extends SwerveChassis {
  /** The CANivore carrying the drivetrain. */
  private static final CANBus kCanBus = new CANBus("OverCANivore");

  /** Drive reduction, shared by the real module config and the simulated drivetrain below. */
  private static final double kDriveGearRatio = 7.03;

  /** Steer reduction, likewise shared. */
  private static final double kTurnGearRatio = 287.0 / 11.0;

  private static final double kWheelRadiusInches = 2.0;
  private static final double kTrackXInches = 11.125;
  private static final double kTrackYInches = 10.875;

  private final OverPigeon pigeon = new OverPigeon(13, kCanBus);

  private final SwerveModule frontLeftModule = new SwerveModule(frontLeftConfig());
  private final SwerveModule frontRightModule = new SwerveModule(frontRightConfig());
  private final SwerveModule backLeftModule = new SwerveModule(backLeftConfig());
  private final SwerveModule backRightModule = new SwerveModule(backRightConfig());

  // Front left, front right, back left, back right. Every list of four in the library is in this
  // order.
  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(
          new Translation2d(
              Units.inchesToMeters(kTrackXInches), Units.inchesToMeters(kTrackYInches)),
          new Translation2d(
              Units.inchesToMeters(kTrackXInches), Units.inchesToMeters(-kTrackYInches)),
          new Translation2d(
              Units.inchesToMeters(-kTrackXInches), Units.inchesToMeters(kTrackYInches)),
          new Translation2d(
              Units.inchesToMeters(-kTrackXInches), Units.inchesToMeters(-kTrackYInches)));

  /**
   * Drive feedforward: ks volts, kv volts per m/s, ka volts per m/s^2.
   *
   * <p>One factory for the four modules, so tuning it cannot end up tuning a single wheel.
   *
   * @return the drive feedforward
   */
  private static SimpleMotorFeedforward driveFeedForward() {
    return new SimpleMotorFeedforward(0.0, 2.0879, 0.098433);
  }

  /** Builds the drivetrain. */
  public Chassis() {
    super();
    configureSwerveBase();
    setAcceptingVisionMeasurements(true);
    resetHeading();
  }

  /**
   * Describes this drivetrain to the physics simulation.
   *
   * <p>The gearing constants are the same ones the module configs use below. That is the entire
   * point: kept anywhere else, a drive ratio can sit at 6.03 against a robot geared 7.03 without
   * anyone noticing.
   */
  @Override
  protected DriveTrainSimulationConfig getSimulationConfig() {
    return DriveTrainSimulationConfig.Default()
        .withRobotMass(Kilograms.of(61.235))
        .withGyro(COTS.ofPigeon2())
        .withSwerveModule(
            new SwerveModuleSimulationConfig(
                DCMotor.getKrakenX60(1),
                DCMotor.getFalcon500(1),
                kDriveGearRatio,
                kTurnGearRatio,
                Volts.of(0.01),
                Volts.of(0.01),
                Inches.of(kWheelRadiusInches),
                KilogramSquareMeters.of(0.03),
                // Conservative tyre friction. The COTS value was flagged as abnormal by the
                // simulation (~2.106); 0.9 is realistic for rubber on carpet.
                0.9))
        .withCustomModuleTranslations(
            new Translation2d[] {
              new Translation2d(Inches.of(kTrackXInches), Inches.of(kTrackYInches)),
              new Translation2d(Inches.of(kTrackXInches), Inches.of(-kTrackYInches)),
              new Translation2d(Inches.of(-kTrackXInches), Inches.of(kTrackYInches)),
              new Translation2d(Inches.of(-kTrackXInches), Inches.of(-kTrackYInches))
            })
        .withBumperSize(Inches.of(30.5), Inches.of(31.5));
  }

  @Override
  protected Pigeon2 getSimulationPigeon() {
    return pigeon;
  }

  @Override
  public double getMaxModuleSpeed() {
    return 4.541;
  }

  @Override
  protected SwerveModule getFrontLeftModule() {
    return frontLeftModule;
  }

  @Override
  protected SwerveModule getFrontRightModule() {
    return frontRightModule;
  }

  @Override
  protected SwerveModule getBackLeftModule() {
    return backLeftModule;
  }

  @Override
  protected SwerveModule getBackRightModule() {
    return backRightModule;
  }

  @Override
  protected SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  @Override
  public Rotation2d getRotation2d() {
    return pigeon.getRotation2d();
  }

  private static SwerveModuleConfig baseModuleConfig() {
    SwerveModuleConfig config = new SwerveModuleConfig(driveFeedForward(), 0, 0);
    config.CanBus = kCanBus;
    config.DriveGearRatio = kDriveGearRatio;
    config.TurnGearRatio = kTurnGearRatio;
    config.WheelDiameter = Units.inchesToMeters(2.0 * kWheelRadiusInches);
    config.DriveMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 30.0;
    config.DriveMotorConfig.CurrentLimits.SupplyCurrentLimit = 60.0;
    config.TurnMotorConfig.Slot0.withKP(40).withKS(0.15);
    config.TurnMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.DriveMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    return config;
  }

  private static SwerveModuleConfig frontLeftConfig() {
    SwerveModuleConfig config = baseModuleConfig();
    config.driveMotorId = 4;
    config.turnMotorId = 3;
    config.EncoderConfig.CanCoderId = 12;
    config.EncoderConfig.Offset = -0.147705078125;
    config.ModuleName = "Front Left";
    return config;
  }

  private static SwerveModuleConfig frontRightConfig() {
    SwerveModuleConfig config = baseModuleConfig();
    config.driveMotorId = 8;
    config.turnMotorId = 7;
    config.EncoderConfig.CanCoderId = 11;
    config.EncoderConfig.Offset = 0.116943359375;
    config.ModuleName = "Front Right";
    return config;
  }

  private static SwerveModuleConfig backLeftConfig() {
    SwerveModuleConfig config = baseModuleConfig();
    config.driveMotorId = 2;
    config.turnMotorId = 1;
    config.EncoderConfig.CanCoderId = 9;
    config.EncoderConfig.Offset = -0.035888671875;
    config.ModuleName = "Back Left";
    return config;
  }

  private static SwerveModuleConfig backRightConfig() {
    SwerveModuleConfig config = baseModuleConfig();
    config.driveMotorId = 6;
    config.turnMotorId = 5;
    config.EncoderConfig.CanCoderId = 10;
    config.EncoderConfig.Offset = 0.37744140625;
    config.ModuleName = "Back Right";
    return config;
  }
}
