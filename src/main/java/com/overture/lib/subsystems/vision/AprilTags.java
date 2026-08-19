// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.subsystems.vision;

import com.overture.lib.simulation.SimPhotonVisionManager;
import com.overture.lib.subsystems.swerve.SwerveChassis;
import com.overture.lib.utils.Logging;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Fuses AprilTag pose estimates from either PhotonVision or a Limelight into the chassis. */
public class AprilTags extends SubsystemBase {
  /** Which coprocessor produces the pose estimates. */
  public enum VisionBackend {
    /** A PhotonVision coprocessor. */
    PhotonVision,
    /** A Limelight. */
    Limelight
  }

  /** Which Limelight botpose flavour to consume. */
  public enum LimelightMode {
    /** Rotation observed from tag geometry. */
    MegaTag1,
    /** Rotation supplied by us, translation from the tags. */
    MegaTag2
  }

  /** Configuration bundle for {@link AprilTags}. */
  public static class Config {
    /** Which backend produces the estimates. */
    public VisionBackend backend = VisionBackend.PhotonVision;

    /** Name of the camera, as configured on the coprocessor. */
    public String cameraName = "";

    /**
     * Supplies the current robot to camera transform, which may move.
     *
     * <p>Left null on purpose, mirroring the empty std::function the C++ Config defaults to. On the
     * Limelight backend the null check is what decides whether we push a camera pose down to the
     * camera at all: leaving this unset means the offset configured in the Limelight web UI is kept
     * instead of being overwritten with zeros. On the PhotonVision backend a transform is required,
     * and leaving this unset fails loudly, exactly as the C++ threw std::bad_function_call.
     */
    public Supplier<Transform3d> cameraToRobotSupplier = null;

    /** Which Limelight botpose flavour to consume. */
    public LimelightMode limelightMode = LimelightMode.MegaTag2;

    /** Maximum trusted distance, in meters, keyed by visible tag count. */
    public Map<Integer, Double> tagValidDistances = defaultTagValidDistances();

    /** Standard deviations used when a single tag is visible. */
    public double[] singleTagStdDevs = {2.0, 2.0, 2.0};

    /** Standard deviations used when several tags are visible. */
    public double[] multiTagStdDevs = {0.07, 0.07, 0.5};

    /**
     * Limelight MegaTag2 complementary filter alpha (mode 4).
     *
     * <p>Higher = LL internal IMU tracks the external (chassis) yaw faster, so corrections from
     * resetHeading / Photon propagate quickly. Default LL value is 0.001; we bump it slightly for
     * faster recovery.
     */
    public double imuAssistAlpha = 0.005;

    /**
     * MegaTag1 yaw watchdog: hard-snap chassis heading when MegaTag1 disagrees with the fused
     * estimator yaw by more than this threshold, in degrees, for {@link #yawCorrectionMinStreak}
     * consecutive frames, while moving slower than {@link #yawCorrectionMaxSpeed}.
     */
    public double yawCorrectionThreshold = 5.0;

    /** Minimum tag count for the yaw watchdog to act. */
    public int yawCorrectionMinTags = 2;

    /** Consecutive disagreeing frames required before the yaw watchdog acts. */
    public int yawCorrectionMinStreak = 5;

    /** Maximum chassis speed, in meters per second, for the yaw watchdog to act. */
    public double yawCorrectionMaxSpeed = 0.5;

    private static Map<Integer, Double> defaultTagValidDistances() {
      Map<Integer, Double> distances = new HashMap<>();
      distances.put(1, 3.5);
      distances.put(2, 6.0);
      distances.put(3, 8.0);
      return distances;
    }
  }

  /* PhotonVision */
  private PhotonCamera camera;
  private PhotonPoseEstimator poseEstimator;
  private PhotonCameraSim cameraSim;

  private final AprilTagFieldLayout tagLayout;
  private final SwerveChassis chassis;
  private final Config config;
  private boolean enabled = true;
  private boolean lastRobotEnabled = false;
  private int yawErrorStreak = 0;
  private final StructArrayPublisher<Pose3d> targetPosesPublisher;
  private final StructPublisher<Pose2d> visionPose2dPublisher;

  /**
   * Constructs an AprilTags subsystem.
   *
   * @param tagLayout the field tag layout
   * @param chassis the chassis to feed measurements into
   * @param config the camera configuration
   */
  public AprilTags(AprilTagFieldLayout tagLayout, SwerveChassis chassis, Config config) {
    this.config = config;
    this.tagLayout = tagLayout;
    this.chassis = chassis;

    NetworkTable cameraTable =
        NetworkTableInstance.getDefault().getTable("AprilTags/" + config.cameraName);
    targetPosesPublisher = cameraTable.getStructArrayTopic("TargetPoses", Pose3d.struct).publish();
    visionPose2dPublisher = cameraTable.getStructTopic("VisionPose2d", Pose2d.struct).publish();

    if (config.backend == VisionBackend.PhotonVision) {
      camera = new PhotonCamera(this.config.cameraName);
      poseEstimator =
          new PhotonPoseEstimator(this.tagLayout, this.config.cameraToRobotSupplier.get());

      if (RobotBase.isSimulation()) {
        cameraSim = new PhotonCameraSim(camera);
        SimPhotonVisionManager.getInstance()
            .addSimCamera(cameraSim, config.cameraToRobotSupplier.get());
      }
    } else if (config.backend == VisionBackend.Limelight) {
      // Set camera pose from the transform supplier
      if (config.cameraToRobotSupplier != null) {
        Transform3d transform = config.cameraToRobotSupplier.get();
        LimelightHelpers.setCameraPose_RobotSpace(
            config.cameraName,
            transform.getX(),
            -transform.getY(),
            transform.getZ(),
            -Units.radiansToDegrees(transform.getRotation().getX()),
            -Units.radiansToDegrees(transform.getRotation().getY()),
            -Units.radiansToDegrees(transform.getRotation().getZ()));
      }

      // Seed internal IMU with external gyro (mode 1) until enabled
      LimelightHelpers.SetIMUMode(config.cameraName, 1);
      // Speed up mode-4 complementary filter so external-yaw corrections
      // (e.g. from Photon multi-tag or our MT1 watchdog) propagate quickly
      LimelightHelpers.SetIMUAssistAlpha(config.cameraName, config.imuAssistAlpha);
    }
  }

  /**
   * Computes the standard deviations to trust a measurement with.
   *
   * @param tagCount how many tags the estimate was built from
   * @param avgDist the average distance to those tags, in meters
   * @param estimatedPose the estimated pose
   * @param trustRotation whether the rotation of the estimate is independent of our own yaw
   * @return the standard deviations
   */
  public Matrix<N3, N1> getEstimationStdDevs(
      int tagCount, double avgDist, Pose2d estimatedPose, boolean trustRotation) {
    double[] estStdDevs = config.singleTagStdDevs.clone();

    if (tagCount == 0) {
      return VecBuilder.fill(estStdDevs[0], estStdDevs[1], estStdDevs[2]);
    }
    if (tagCount > 1) {
      estStdDevs = config.multiTagStdDevs.clone();
    }
    if (tagCount == 1 && avgDist > 4.0) {
      estStdDevs = new double[] {1e6, 1e6, 1e6};
    } else {
      double scale = 1 + (avgDist * avgDist / 30);
      estStdDevs =
          new double[] {estStdDevs[0] * scale, estStdDevs[1] * scale, estStdDevs[2] * scale};
    }
    // MegaTag2's rotation is just the yaw we sent it via SetRobotOrientation,
    // so feeding it back into the estimator's theta would create a loop.
    // Tell the pose estimator to ignore rotation entirely for those samples.
    if (!trustRotation) {
      estStdDevs[2] = 9999999.0;
    }
    return VecBuilder.fill(estStdDevs[0], estStdDevs[1], estStdDevs[2]);
  }

  /**
   * Computes the standard deviations to trust a measurement with, trusting its rotation.
   *
   * @param tagCount how many tags the estimate was built from
   * @param avgDist the average distance to those tags, in meters
   * @param estimatedPose the estimated pose
   * @return the standard deviations
   */
  public Matrix<N3, N1> getEstimationStdDevs(int tagCount, double avgDist, Pose2d estimatedPose) {
    return getEstimationStdDevs(tagCount, avgDist, estimatedPose, true);
  }

  /**
   * Feeds a measurement into the chassis pose estimator.
   *
   * @param pose the vision estimated pose
   * @param timestamp the timestamp of the sample, in seconds
   * @param tagCount how many tags the estimate was built from
   * @param avgDist the average distance to those tags, in meters
   * @param trustRotation whether the rotation of the estimate is independent of our own yaw
   */
  public void addMeasurementToChassis(
      Pose2d pose, double timestamp, int tagCount, double avgDist, boolean trustRotation) {
    chassis.addVisionMeasurement(
        pose, timestamp, getEstimationStdDevs(tagCount, avgDist, pose, trustRotation));

    Logging.logPose2d(
        "/Swerve/Vision/" + config.cameraName, pose, Timer.getFPGATimestamp() - timestamp);
    visionPose2dPublisher.set(pose);
  }

  /**
   * Feeds a measurement into the chassis pose estimator, trusting its rotation.
   *
   * @param pose the vision estimated pose
   * @param timestamp the timestamp of the sample, in seconds
   * @param tagCount how many tags the estimate was built from
   * @param avgDist the average distance to those tags, in meters
   */
  public void addMeasurementToChassis(Pose2d pose, double timestamp, int tagCount, double avgDist) {
    addMeasurementToChassis(pose, timestamp, tagCount, avgDist, true);
  }

  /**
   * Enables or disables this camera.
   *
   * @param enabled whether the camera contributes measurements
   */
  public void setEnabled(boolean enabled) {
    this.enabled = enabled;
  }

  @Override
  public void periodic() {
    if (!enabled) {
      return;
    }

    switch (config.backend) {
      case PhotonVision:
        periodicPhotonVision();
        break;
      case Limelight:
        periodicLimelight();
        break;
      default:
        break;
    }
  }

  private void periodicPhotonVision() {
    poseEstimator.setRobotToCameraTransform(config.cameraToRobotSupplier.get());

    for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
      Optional<EstimatedRobotPose> visionEst = poseEstimator.estimateCoprocMultiTagPose(result);
      if (visionEst.isEmpty()) {
        visionEst = poseEstimator.estimateLowestAmbiguityPose(result);
      }

      if (visionEst.isPresent()) {
        Pose2d poseTo2d = visionEst.get().estimatedPose.toPose2d();

        List<PhotonTrackedTarget> targets = result.getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (PhotonTrackedTarget tgt : targets) {
          Optional<Pose3d> tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
          if (tagPose.isPresent()) {
            numTags++;
            avgDist +=
                tagPose.get().toPose2d().getTranslation().getDistance(poseTo2d.getTranslation());
          }
        }
        if (numTags > 0) {
          avgDist /= numTags;
        }

        // Publish target poses for visualization
        List<Pose3d> targetPoses = new ArrayList<>();
        Pose3d current3d;
        if (RobotBase.isSimulation()) {
          current3d = SimPhotonVisionManager.getInstance().getRobotPose();
        } else {
          current3d = new Pose3d(chassis.getEstimatedPose());
        }
        for (PhotonTrackedTarget t : targets) {
          targetPoses.add(
              current3d
                  .transformBy(config.cameraToRobotSupplier.get())
                  .transformBy(t.getBestCameraToTarget()));
        }
        targetPosesPublisher.set(targetPoses.toArray(new Pose3d[0]));

        addMeasurementToChassis(poseTo2d, visionEst.get().timestampSeconds, numTags, avgDist);
      } else {
        targetPosesPublisher.set(new Pose3d[0]);
      }
    }
  }

  private void periodicLimelight() {
    // Switch IMU mode and throttle based on robot state
    // Mode 1: seed internal IMU when robot is disabled (pre-match)
    // Mode 4: internal IMU + external assist when robot is enabled (recommended)
    boolean robotEnabled = DriverStation.isEnabled();
    if (robotEnabled != lastRobotEnabled) {
      LimelightHelpers.SetIMUMode(config.cameraName, robotEnabled ? 4 : 1);
      LimelightHelpers.SetThrottle(config.cameraName, robotEnabled ? 0 : 200);
      // Re-assert alpha in case the LL rebooted between transitions
      LimelightHelpers.SetIMUAssistAlpha(config.cameraName, config.imuAssistAlpha);
      lastRobotEnabled = robotEnabled;
      yawErrorStreak = 0;
    }

    // MegaTag2 requires robot orientation every frame
    double robotYaw = chassis.getEstimatedPose().getRotation().getDegrees();
    LimelightHelpers.SetRobotOrientation(config.cameraName, robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);

    if (config.limelightMode == LimelightMode.MegaTag2 && robotEnabled) {
      // MegaTag1 yaw watchdog: snap chassis heading if it has drifted away
      // from a high-confidence MT1 reading. Debounced and gated on low
      // chassis speed because MT1 yaw is only trustworthy near-stationary.
      LimelightHelpers.PoseEstimate mt1Estimate =
          LimelightHelpers.getBotPoseEstimate_wpiBlue(config.cameraName);

      ChassisSpeeds speeds = chassis.getCurrentSpeeds();
      double speedMag = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

      boolean watchdogEligible =
          mt1Estimate != null
              && mt1Estimate.tagCount >= config.yawCorrectionMinTags
              && speedMag < config.yawCorrectionMaxSpeed;

      if (watchdogEligible) {
        double mt1Yaw = mt1Estimate.pose.getRotation().getDegrees();
        double chassisYaw = chassis.getEstimatedPose().getRotation().getDegrees();
        double yawError = Math.abs(mt1Yaw - chassisYaw);

        // Normalize to [0, 180]
        if (yawError > 180.0) {
          yawError = 360.0 - yawError;
        }

        if (yawError > config.yawCorrectionThreshold) {
          if (++yawErrorStreak >= config.yawCorrectionMinStreak) {
            // Hard-snap heading. resetHeading rewrites the pose
            // estimator's gyro offset without touching translation,
            // so MT2 will produce a correct pose on the very next
            // frame instead of fighting a Kalman blend.
            chassis.resetHeading(mt1Yaw);
            yawErrorStreak = 0;
            return;
          }
        } else {
          yawErrorStreak = 0;
        }
      } else {
        yawErrorStreak = 0;
      }

      // Normal MegaTag2 path. Theta std dev is forced to infinity inside
      // addMeasurementToChassis because MT2's rotation is just the yaw we
      // sent in via SetRobotOrientation; trusting it would feed our own
      // yaw back into ourselves.
      LimelightHelpers.PoseEstimate estimate =
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(config.cameraName);

      if (estimate == null || estimate.tagCount == 0) {
        targetPosesPublisher.set(new Pose3d[0]);
        return;
      }

      addMeasurementToChassis(
          estimate.pose, estimate.timestampSeconds, estimate.tagCount, estimate.avgTagDist, false);
    } else {
      // MegaTag1 mode or disabled: use MegaTag1. MT1 rotation is observed
      // from tag geometry, so it's safe to trust in the estimator.
      LimelightHelpers.PoseEstimate estimate =
          LimelightHelpers.getBotPoseEstimate_wpiBlue(config.cameraName);

      if (estimate == null || estimate.tagCount == 0) {
        targetPosesPublisher.set(new Pose3d[0]);
        return;
      }

      addMeasurementToChassis(
          estimate.pose, estimate.timestampSeconds, estimate.tagCount, estimate.avgTagDist);
    }
  }
}
