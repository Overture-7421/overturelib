package com.overture.lib.subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

/**
 * The AprilTags class
 *
 * <p>AprilTags is a class that contains the PhotonCamera and PhotonPoseEstimator classes for pose
 * estimation.
 */
public class AprilTags {
  PhotonCamera camera;
  PhotonPoseEstimator poseEstimator;

  AprilTagFieldLayout tagLayout;
  AprilTagsConfig config;

  /**
   * Constructor for AprilTags
   *
   * @param tagLayout The tag layout
   * @param config The configuration
   */
  public AprilTags(AprilTagFieldLayout tagLayout, AprilTagsConfig config) {
    this.tagLayout = tagLayout;
    this.config = config;

    camera = new PhotonCamera(config.getCameraName());
    poseEstimator =
        new PhotonPoseEstimator(
            tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, config.getCameraToRobot());
  }
}
