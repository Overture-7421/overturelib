package com.overture.lib.math;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.Time;

/**
 * The ChassisAccels class
 *
 * <p>ChassisAccels is a class that contains the LinearAcceleration in the x and y directions and
 * the AngularAcceleration of the robot.
 */
public class ChassisAccels {
  /** The LinearAcceleration in the x direction. */
  public LinearAcceleration ax;
  /** The LinearAcceleration in the y direction. */
  public LinearAcceleration ay;
  /** The AngularAcceleration. */
  public AngularAcceleration omega;

  /**
   * Constructor for ChassisAccels, initializes the ChassisAccels with the given ChassisSpeeds and
   * Time.
   *
   * @param currentSpeeds The current ChassisSpeeds of the robot.
   * @param lastSpeeds The last ChassisSpeeds of the robot.
   * @param period The update period of the robot in seconds.
   */
  public ChassisAccels(ChassisSpeeds currentSpeeds, ChassisSpeeds lastSpeeds, Time period) {
    currentSpeeds.minus(lastSpeeds);
    ax = MetersPerSecondPerSecond.of(currentSpeeds.vxMetersPerSecond / period.magnitude());
    ay = MetersPerSecondPerSecond.of(currentSpeeds.vyMetersPerSecond / period.magnitude());
    omega = RadiansPerSecondPerSecond.of(currentSpeeds.omegaRadiansPerSecond / period.magnitude());

    if (ax.abs(MetersPerSecondPerSecond) > 6.0) {
      ax = MetersPerSecondPerSecond.of(6.0 * Math.signum(ax.magnitude()));
    }

    if (ay.abs(MetersPerSecondPerSecond) > 6.0) {
      ay = MetersPerSecondPerSecond.of(6.0 * Math.signum(ay.magnitude()));
    }

    if (omega.abs(RadiansPerSecondPerSecond) > 6.0) {
      omega = RadiansPerSecondPerSecond.of(6.0 * Math.signum(omega.magnitude()));
    }
  }

  /**
   * Constructor for ChassisAccels, initializes the ChassisAccels with the given LinearAcceleration
   * and AngularAcceleration.
   *
   * @param ax The LinearAcceleration in the x direction.
   * @param ay The LinearAcceleration in the y direction.
   * @param omega The AngularAcceleration.
   */
  ChassisAccels(LinearAcceleration ax, LinearAcceleration ay, AngularAcceleration omega) {
    this.ax = ax;
    this.ay = ay;
    this.omega = omega;
  }

  /** Constructor for ChassisAccels, initializes the ChassisAccels with zeros. */
  ChassisAccels() {
    ax = MetersPerSecondPerSecond.zero();
    ay = MetersPerSecondPerSecond.zero();
    omega = RadiansPerSecondPerSecond.zero();
  }

  /**
   * Returns the ChassisAccels in Robot Relative Accels.
   *
   * @param ax The LinearAcceleration in the x direction.
   * @param ay The LinearAcceleration in the y direction.
   * @param omega The AngularAcceleration.
   * @param robotAngle The Rotation2d of the robot.
   * @return The ChassisAccels in Robot Relative Accels.
   */
  static ChassisAccels FromRobotRelativeAccels(
      LinearAcceleration ax,
      LinearAcceleration ay,
      AngularAcceleration omega,
      Rotation2d robotAngle) {
    Translation2d rotated =
        new Translation2d(ax.baseUnitMagnitude(), ay.baseUnitMagnitude()).rotateBy(robotAngle);
    return new ChassisAccels(
        MetersPerSecondPerSecond.of(rotated.getX()),
        MetersPerSecondPerSecond.of(rotated.getY()),
        omega);
  }

  /**
   * Returns the ChassisAccels in Robot Relative Accels.
   *
   * @param robotRelativAccels The ChassisAccels in Robot Relative Accels.
   * @param robotAngle The Rotation2d of the robot.
   * @return The ChassisAccels in Robot Relative Accels.
   */
  static ChassisAccels FromRobotRelativeAccels(
      ChassisAccels robotRelativAccels, Rotation2d robotAngle) {
    return FromRobotRelativeAccels(
        robotRelativAccels.ax, robotRelativAccels.ay, robotRelativAccels.omega, robotAngle);
  }
}
