package com.overture.lib.subsystems.Vision;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Distance;
import java.util.Map;

/**
 * The AprilTagsConfig class
 *
 * <p>AprilTagsConfig is a class that contains the configuration for the AprilTags class.
 */
public class AprilTagsConfig {
  /** The camera name */
  private String CameraName;
  /** The camera to robot transform */
  private Transform3d CameraToRobot;
  /** The tag valid distance */
  private Map<Integer, Distance> tagValidDistance;

  /** Constructor for AprilTagsConfig */
  public AprilTagsConfig() {
    CameraName = "";
    CameraToRobot = new Transform3d();
    tagValidDistance =
        Map.ofEntries(
            Map.entry(1, Meters.of(3.5)), Map.entry(2, Meters.of(6)), Map.entry(3, Meters.of(8)));
  }

  /**
   * Sets the camera name
   *
   * @param CameraName The camera name
   * @return The AprilTagsConfig object
   */
  public AprilTagsConfig withCameraName(String CameraName) {
    this.CameraName = CameraName;
    return this;
  }

  /**
   * Sets the camera to robot transform
   *
   * @param CameraToRobot The camera to robot transform
   * @return The AprilTagsConfig object
   */
  public AprilTagsConfig withCameraToRobot(Transform3d CameraToRobot) {
    this.CameraToRobot = CameraToRobot;
    return this;
  }

  /**
   * Sets the tag valid distance
   *
   * @param tagValidDistance The tag valid distance
   * @return The AprilTagsConfig object
   */
  public AprilTagsConfig withTagValidDistance(Map<Integer, Distance> tagValidDistance) {
    this.tagValidDistance = tagValidDistance;
    return this;
  }

  /**
   * Gets the camera name
   *
   * @return The camera name
   */
  public String getCameraName() {
    return CameraName;
  }

  /**
   * Gets the camera to robot transform
   *
   * @return The camera to robot transform
   */
  public Transform3d getCameraToRobot() {
    return CameraToRobot;
  }

  /**
   * Gets the tag valid distance
   *
   * @return The tag valid distance
   */
  public Map<Integer, Distance> getTagValidDistance() {
    return tagValidDistance;
  }
}
