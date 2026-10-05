// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Chassis;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;

/**
 * Hands the chassis to BLine and builds the commands that follow its paths.
 *
 * <p>This is robot code. OvertureLib does not know BLine exists, so this class is the only place
 * the two meet. Build one of these in RobotContainer and ask it for every path command.
 */
public final class BLinePaths {
  private final Chassis chassis;
  private final FollowPath.Builder builder;

  /**
   * Wires BLine to the chassis.
   *
   * @param chassis the drivetrain BLine will drive
   */
  public BLinePaths(Chassis chassis) {
    this.chassis = chassis;

    // The gains are BLine's own starting values, not ones tuned on Shelby.
    builder =
        new FollowPath.Builder(
                chassis,
                chassis::getEstimatedPose,
                // Robot relative, both the speeds read and the speeds commanded.
                chassis::getCurrentSpeeds,
                chassis::setTargetSpeeds,
                // Translation: how hard to drive at what is left of the path.
                new PIDController(5.0, 0.0, 0.0),
                // Rotation: how hard to turn towards the heading the path asks for.
                new PIDController(3.0, 0.0, 0.0),
                // Cross track: how hard to pull back onto the line between two points.
                new PIDController(2.0, 0.0, 0.0))
            // Paths are drawn on the blue side and flipped when we are red.
            .withDefaultShouldFlip();
  }

  /**
   * Follows a path from wherever the robot believes it is.
   *
   * @param pathName the path file in deploy/autos/paths, without the .json
   * @return the command
   */
  public Command follow(String pathName) {
    // The builder remembers its pose reset between builds, so it is set on every build rather
    // than left to whichever method happened to run last.
    return builder.withPoseReset(pose -> {}).build(new Path(pathName));
  }

  /**
   * Places the robot at the start of a path, then follows it. For the first path of an auto.
   *
   * <p>Only the first. BLine resets the pose every time a command built with a reset starts, so
   * using this for a later path throws away whatever odometry and vision worked out on the way
   * there.
   *
   * @param pathName the path file in deploy/autos/paths, without the .json
   * @return the command
   */
  public Command resetAndFollow(String pathName) {
    // resetPose rather than resetOdometry: it places the simulated robot there as well.
    return builder.withPoseReset(chassis::resetPose).build(new Path(pathName));
  }
}
