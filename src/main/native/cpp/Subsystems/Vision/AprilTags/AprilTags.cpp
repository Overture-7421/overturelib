// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "OvertureLib/Subsystems/Vision/AprilTags/AprilTags.h"
#include "OvertureLib/Simulation/SimPhotonVisionManager/SimPhotonVisionManager.h"

AprilTags::AprilTags(frc::AprilTagFieldLayout *tagLayout,
		SwerveChassis *chassis, Config configIn) : tagLayout(tagLayout), chassis(
		chassis), config(std::move(configIn)) {
	camera = std::make_unique < photon::PhotonCamera
			> (this->config.cameraName);
	poseEstimator = std::make_unique < photon::PhotonPoseEstimator
			> (*this->tagLayout, this->config.cameraToRobot);

	auto cameraTable = nt::NetworkTableInstance::GetDefault().GetTable(
			"AprilTags/" + this->config.cameraName);
	targetPosesPublisher = cameraTable->GetStructArrayTopic < frc::Pose3d
			> ("TargetPoses").Publish();
	visionPose2dPublisher = cameraTable->GetStructTopic < frc::Pose2d
			> ("VisionPose2d").Publish();
#ifndef __FRC_ROBORIO__ // If on simulation
	cameraSim = std::make_shared < photon::PhotonCameraSim > (camera.get());
	SimPhotonVisionManager::GetInstance().AddSimCamera(cameraSim.get(),
			this->config.cameraToRobot);
#endif

	singleTagStdDevs = this->config.singleTagStdDevs;
	multiTagStdDevs = this->config.multiTagStdDevs;
}

wpi::array<double, 3> AprilTags::GetEstimationStdDevs(
		const photon::PhotonPipelineResult &result, frc::Pose2d estimatedPose) {
	wpi::array<double, 3> estStdDevs = singleTagStdDevs;
	auto targets = result.GetTargets();
	int numTags = 0;
	units::meter_t avgDist = 0_m;
	for (const auto &tgt : targets) {
		auto tagPose = poseEstimator->GetFieldLayout().GetTagPose(
				tgt.GetFiducialId());
		if (tagPose) {
			numTags++;
			avgDist += tagPose->ToPose2d().Translation().Distance(
					estimatedPose.Translation());
		}
	}
	if (numTags == 0) {
		return estStdDevs;
	}
	avgDist /= numTags;
	if (numTags > 1) {
		estStdDevs = multiTagStdDevs;
	}
	if (numTags == 1 && avgDist > 4_m) {
		estStdDevs = wpi::array<double, 3> { 1e6, 1e6, 1e6 };
	} else {
		double scaleFactor = 1 + (avgDist.value() * avgDist.value() / 30);
		for (size_t i = 0; i < estStdDevs.size(); i++) {
			estStdDevs[i] *= scaleFactor;
		}
	}
	return estStdDevs;
}

void AprilTags::addMeasurementToChassis(
		const photon::PhotonPipelineResult &result, frc::Pose2d pose,
		units::second_t timestamp) {

	std::vector < frc::Pose3d > targets;
	frc::Pose3d current3d;

#ifndef __FRC_ROBORIO__ // If on simulation
	current3d = SimPhotonVisionManager::GetInstance().GetRobotPose();
#else
	current3d = frc::Pose3d(chassis->getEstimatedPose());
#endif

	for (const auto &t : result.GetTargets()) {
		targets.push_back(
				current3d.TransformBy(
						poseEstimator->GetRobotToCameraTransform()).TransformBy(
						t.GetBestCameraToTarget()));
	}
	targetPosesPublisher.Set(targets);
	chassis->addVisionMeasurement(pose, timestamp,
			GetEstimationStdDevs(result, pose));

	Logging::LogPose2d("/Swerve/Photonvision/" + config.cameraName, pose,
			frc::Timer::GetFPGATimestamp() - timestamp);
	visionPose2dPublisher.Set(pose);
}

void AprilTags::setEnabled(bool enabled) {
	this->enabled = enabled;
}

std::optional<photon::EstimatedRobotPose> AprilTags::getEstimatedPose(
		const photon::PhotonPipelineResult &result) {
	auto visionEst = poseEstimator->EstimateCoprocMultiTagPose(result);
	if (!visionEst) {
		visionEst = poseEstimator->EstimateLowestAmbiguityPose(result);
	}
	return visionEst;
}

void AprilTags::Periodic() {
	if (enabled) {
		for (const auto &result : camera->GetAllUnreadResults()) {
			auto visionEst = getEstimatedPose(result);
			m_latestResult = result;

			if (visionEst) {
				frc::Pose2d poseTo2d =
						visionEst.value().estimatedPose.ToPose2d();
				addMeasurementToChassis(result, poseTo2d,
						visionEst.value().timestamp);
			} else {
				targetPosesPublisher.Set(std::vector<frc::Pose3d> { });
				continue;
			}
		}
	}
}

