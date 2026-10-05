// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Chassis;

import com.overture.lib.utils.UtilityFunctions;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Hands the chassis to PathPlanner.
 *
 * <p>This is robot code. OvertureLib does not know PathPlanner exists, so this class is the only
 * place the two meet, and a robot that follows its paths some other way never installs it.
 */
public final class PathPlannerConfig {
  /** Gains correcting position error along a path. */
  private static final PIDConstants kTranslationPID = new PIDConstants(6.0, 0.0, 0.0);

  /** Gains correcting heading error along a path. */
  private static final PIDConstants kRotationPID = new PIDConstants(6.0, 0.0, 0.0);

  private PathPlannerConfig() {}

  /**
   * Configures PathPlanner's AutoBuilder. Call it once, after the chassis is built and before
   * anything asks PathPlanner for an auto, a path command or the auto chooser.
   *
   * @param chassis the drivetrain PathPlanner will drive
   */
  public static void configure(Chassis chassis) {
    // Mass, MOI, module positions and motor limits, read from deploy/pathplanner/settings.json.
    // They are typed into the PathPlanner app by hand, so nothing but care keeps them agreeing
    // with the Chassis.
    RobotConfig robotConfig;
    try {
      robotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      throw new RuntimeException("Failed to load the PathPlanner RobotConfig from GUI settings", e);
    }

    AutoBuilder.configure(
        chassis::getEstimatedPose,
        // resetPose rather than resetOdometry. An auto resetting the pose means "the robot is
        // here now", and resetPose places the simulated robot there as well.
        chassis::resetPose,
        chassis::getCurrentSpeeds,
        // Robot relative. The parameter type is spelled out to pick the overload that takes the
        // speeds alone, without the per module feedforwards.
        (ChassisSpeeds speeds) -> chassis.setTargetSpeeds(speeds),
        new PPHolonomicDriveController(kTranslationPID, kRotationPID),
        robotConfig,
        // Paths are drawn on the blue side and flipped when we are red.
        UtilityFunctions::isRedAlliance,
        chassis);
  }
}
