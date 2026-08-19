// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.simulation;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructSubscriber;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;

/**
 * Drives PhotonVision's simulated cameras from the externally simulated drivetrain pose.
 *
 * <p>The C++ counterpart only existed off the roboRIO, behind {@code #ifndef __FRC_ROBORIO__}. Java
 * has no preprocessor and {@link com.overture.lib.robots.OverRobot} holds this singleton as a plain
 * field, so the class is loaded on a real robot too. Everything expensive is therefore created
 * lazily on first use: {@link VisionSystemSim} publishes a {@code Field2d} to SmartDashboard from
 * its constructor, which would otherwise put a phantom sim widget and its NetworkTables traffic on
 * a competition robot. Every caller of the methods below is already gated on {@code
 * RobotBase.isSimulation()}, so on a real robot none of this is ever built.
 */
public final class SimPhotonVisionManager {
  private static final SimPhotonVisionManager instance = new SimPhotonVisionManager();

  private VisionSystemSim visionSim;
  private StructSubscriber<Pose2d> simulatedDriveTrainPoseEntry;

  private SimPhotonVisionManager() {}

  private VisionSystemSim visionSim() {
    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      System.out.println("SimPhotonVisionManager initialized!!!");
    }
    return visionSim;
  }

  private StructSubscriber<Pose2d> simulatedDriveTrainPoseEntry() {
    if (simulatedDriveTrainPoseEntry == null) {
      simulatedDriveTrainPoseEntry =
          NetworkTableInstance.getDefault()
              .getStructTopic(
                  "/AdvantageKit/RealOutputs/FieldSimulation/RobotPosition", Pose2d.struct)
              .subscribe(new Pose2d());
    }
    return simulatedDriveTrainPoseEntry;
  }

  /**
   * Returns the singleton instance.
   *
   * @return the sim PhotonVision manager
   */
  public static SimPhotonVisionManager getInstance() {
    return instance;
  }

  /**
   * Populates the simulated field with AprilTags.
   *
   * @param tagLayout the ideal tag layout, not the one we measured
   */
  public void init(AprilTagFieldLayout tagLayout) {
    visionSim().addAprilTags(tagLayout);
  }

  /** Moves the simulated cameras to follow the simulated robot. Call this periodically. */
  public void update() {
    StructSubscriber<Pose2d> poseEntry = simulatedDriveTrainPoseEntry();
    if (!poseEntry.exists()) {
      System.err.println(
          "Simulated drive train pose entry does not exist! Cannot update simulated cameras");
    }
    visionSim().update(poseEntry.get());
  }

  /**
   * Adds a simulated camera to the simulated field.
   *
   * @param camera the simulated camera
   * @param robotToCamera the transform from the robot to the camera
   */
  public void addSimCamera(PhotonCameraSim camera, Transform3d robotToCamera) {
    visionSim().addCamera(camera, robotToCamera);
    System.out.println(
        "Added camera " + camera.getCamera().getName() + " to SimPhotonVisionManager");
  }

  /**
   * Returns the pose of the simulated robot.
   *
   * @return the simulated robot pose
   */
  public Pose3d getRobotPose() {
    return visionSim().getRobotPose();
  }
}
