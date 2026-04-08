// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "OvertureLib/Subsystems/Vision/AprilTags/AprilTags.h"
#include "OvertureLib/Subsystems/Swerve/SwerveChassis/SwerveChassis.h"
#include "OvertureLib/Simulation/SimPhotonVisionManager/SimPhotonVisionManager.h"

#include <cmath>

AprilTags::AprilTags(frc::AprilTagFieldLayout *tagLayout,
		SwerveChassis *chassis, Config config) {
	this->config = config;
	this->tagLayout = tagLayout;
	this->chassis = chassis;

	auto cameraTable = nt::NetworkTableInstance::GetDefault().GetTable(
			"AprilTags/" + config.cameraName);
	targetPosesPublisher = cameraTable->GetStructArrayTopic < frc::Pose3d
			> ("TargetPoses").Publish();
	visionPose2dPublisher = cameraTable->GetStructTopic < frc::Pose2d
			> ("VisionPose2d").Publish();

	if (config.backend == VisionBackend::PhotonVision) {
		camera = std::make_unique < photon::PhotonCamera
				> (this->config.cameraName);
		poseEstimator = std::make_unique < photon::PhotonPoseEstimator
				> (*this->tagLayout, this->config.cameraToRobotSupplier());

#ifndef __FRC_ROBORIO__ // If on simulation
		cameraSim = std::make_shared < photon::PhotonCameraSim > (camera.get());
		SimPhotonVisionManager::GetInstance().AddSimCamera(cameraSim.get(),
				config.cameraToRobotSupplier());
#endif
	} else if (config.backend == VisionBackend::Limelight) {
		// Set camera pose from the transform supplier
		if (config.cameraToRobotSupplier) {
			frc::Transform3d transform = config.cameraToRobotSupplier();
			LimelightHelpers::setCameraPose_RobotSpace(config.cameraName,
					transform.X().value(), -transform.Y().value(),
					transform.Z().value(),
					-transform.Rotation().X().convert<units::degrees>().value(),
					-transform.Rotation().Y().convert<units::degrees>().value(),
					-transform.Rotation().Z().convert<units::degrees>().value());
		}

		// Seed internal IMU with external gyro (mode 1) until enabled
		LimelightHelpers::SetIMUMode(config.cameraName, 1);
		// Speed up mode-4 complementary filter so external-yaw corrections
		// (e.g. from Photon multi-tag or our MT1 watchdog) propagate quickly
		LimelightHelpers::SetIMUAssistAlpha(config.cameraName,
				config.imuAssistAlpha);
	}
}

wpi::array<double, 3> AprilTags::GetEstimationStdDevs(int tagCount,
		units::meter_t avgDist, frc::Pose2d estimatedPose, bool trustRotation) {
	wpi::array<double, 3> estStdDevs = config.singleTagStdDevs;

	if (tagCount == 0) {
		return estStdDevs;
	}
	if (tagCount > 1) {
		estStdDevs = config.multiTagStdDevs;
	}
	if (tagCount == 1 && avgDist > 4_m) {
		estStdDevs = wpi::array<double, 3> { 1e6, 1e6, 1e6 };
	} else {
		double scale = 1 + (avgDist.value() * avgDist.value() / 30);
		estStdDevs = wpi::array<double, 3> { estStdDevs[0] * scale,
				estStdDevs[1] * scale, estStdDevs[2] * scale };
	}
	// MegaTag2's rotation is just the yaw we sent it via SetRobotOrientation,
	// so feeding it back into the estimator's theta would create a loop.
	// Tell the pose estimator to ignore rotation entirely for those samples.
	if (!trustRotation) {
		estStdDevs[2] = 9999999.0;
	}
	return estStdDevs;
}

void AprilTags::addMeasurementToChassis(frc::Pose2d pose,
		units::second_t timestamp, int tagCount, units::meter_t avgDist,
		bool trustRotation) {
	chassis->addVisionMeasurement(pose, timestamp,
			GetEstimationStdDevs(tagCount, avgDist, pose, trustRotation));

	Logging::LogPose2d("/Swerve/Vision/" + config.cameraName, pose,
			frc::Timer::GetFPGATimestamp() - timestamp);
	visionPose2dPublisher.Set(pose);
}

void AprilTags::setEnabled(bool enabled) {
	this->enabled = enabled;
}

void AprilTags::Periodic() {
	if (!enabled) {
		return;
	}

	switch (config.backend) {
	case VisionBackend::PhotonVision:
		PeriodicPhotonVision();
		break;
	case VisionBackend::Limelight:
		PeriodicLimelight();
		break;
	}
}

void AprilTags::PeriodicPhotonVision() {
	poseEstimator->SetRobotToCameraTransform(config.cameraToRobotSupplier());

	for (const auto &result : camera->GetAllUnreadResults()) {
		auto visionEst = poseEstimator->EstimateCoprocMultiTagPose(result);
		if (!visionEst) {
			visionEst = poseEstimator->EstimateLowestAmbiguityPose(result);
		}

		if (visionEst) {
			frc::Pose2d poseTo2d = visionEst.value().estimatedPose.ToPose2d();

			auto targets = result.GetTargets();
			int numTags = 0;
			units::meter_t avgDist = 0_m;
			for (const auto &tgt : targets) {
				auto tagPose = poseEstimator->GetFieldLayout().GetTagPose(
						tgt.GetFiducialId());
				if (tagPose) {
					numTags++;
					avgDist += tagPose->ToPose2d().Translation().Distance(
							poseTo2d.Translation());
				}
			}
			if (numTags > 0) {
				avgDist /= numTags;
			}

			// Publish target poses for visualization
			std::vector < frc::Pose3d > targetPoses;
			frc::Pose3d current3d;
#ifndef __FRC_ROBORIO__
			current3d = SimPhotonVisionManager::GetInstance().GetRobotPose();
#else
			current3d = frc::Pose3d(chassis->getEstimatedPose());
#endif
			for (const auto &t : targets) {
				targetPoses.push_back(
						current3d.TransformBy(config.cameraToRobotSupplier()).TransformBy(
								t.GetBestCameraToTarget()));
			}
			targetPosesPublisher.Set(targetPoses);

			addMeasurementToChassis(poseTo2d, visionEst.value().timestamp,
					numTags, avgDist);
		} else {
			targetPosesPublisher.Set(std::vector<frc::Pose3d> { });
		}
	}
}

void AprilTags::PeriodicLimelight() {
	// Switch IMU mode and throttle based on robot state
	// Mode 1: seed internal IMU when robot is disabled (pre-match)
	// Mode 4: internal IMU + external assist when robot is enabled (recommended)
	bool robotEnabled = frc::DriverStation::IsEnabled();
	if (robotEnabled != lastRobotEnabled) {
		LimelightHelpers::SetIMUMode(config.cameraName, robotEnabled ? 4 : 1);
		LimelightHelpers::SetThrottle(config.cameraName,
				robotEnabled ? 0 : 200);
		// Re-assert alpha in case the LL rebooted between transitions
		LimelightHelpers::SetIMUAssistAlpha(config.cameraName,
				config.imuAssistAlpha);
		lastRobotEnabled = robotEnabled;
		yawErrorStreak = 0;
	}

	// MegaTag2 requires robot orientation every frame
	double robotYaw = chassis->getEstimatedPose().Rotation().Degrees().value();
	LimelightHelpers::SetRobotOrientation(config.cameraName, robotYaw, 0.0, 0.0,
			0.0, 0.0, 0.0);

	if (config.limelightMode == LimelightMode::MegaTag2 && robotEnabled) {
		// MegaTag1 yaw watchdog: snap chassis heading if it has drifted away
		// from a high-confidence MT1 reading. Debounced and gated on low
		// chassis speed because MT1 yaw is only trustworthy near-stationary.
		LimelightHelpers::PoseEstimate mt1Estimate =
				LimelightHelpers::getBotPoseEstimate_wpiBlue(config.cameraName);

		frc::ChassisSpeeds speeds = chassis->getCurrentSpeeds();
		units::meters_per_second_t speedMag { std::hypot(speeds.vx.value(),
				speeds.vy.value()) };

		bool watchdogEligible = mt1Estimate.tagCount
				>= config.yawCorrectionMinTags
				&& speedMag < config.yawCorrectionMaxSpeed;

		if (watchdogEligible) {
			units::degree_t mt1Yaw = mt1Estimate.pose.Rotation().Degrees();
			units::degree_t chassisYaw =
					chassis->getEstimatedPose().Rotation().Degrees();
			units::degree_t yawError = units::math::abs(mt1Yaw - chassisYaw);

			// Normalize to [0, 180]
			if (yawError > 180_deg) {
				yawError = 360_deg - yawError;
			}

			if (yawError > config.yawCorrectionThreshold) {
				if (++yawErrorStreak >= config.yawCorrectionMinStreak) {
					// Hard-snap heading. resetHeading rewrites the pose
					// estimator's gyro offset without touching translation,
					// so MT2 will produce a correct pose on the very next
					// frame instead of fighting a Kalman blend.
					chassis->resetHeading(mt1Yaw);
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
		LimelightHelpers::PoseEstimate estimate =
				LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2(
						config.cameraName);

		if (estimate.tagCount == 0) {
			targetPosesPublisher.Set(std::vector<frc::Pose3d> { });
			return;
		}

		addMeasurementToChassis(estimate.pose, estimate.timestampSeconds,
				estimate.tagCount, units::meter_t { estimate.avgTagDist },
				false);
	} else {
		// MegaTag1 mode or disabled: use MegaTag1. MT1 rotation is observed
		// from tag geometry, so it's safe to trust in the estimator.
		LimelightHelpers::PoseEstimate estimate =
				LimelightHelpers::getBotPoseEstimate_wpiBlue(config.cameraName);

		if (estimate.tagCount == 0) {
			targetPosesPublisher.Set(std::vector<frc::Pose3d> { });
			return;
		}

		addMeasurementToChassis(estimate.pose, estimate.timestampSeconds,
				estimate.tagCount, units::meter_t { estimate.avgTagDist });
	}
}
