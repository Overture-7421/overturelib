// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "OvertureLib/Subsystems/Vision/AprilTags/AprilTags.h"
#include <iostream>

AprilTags::AprilTags(frc::AprilTagFieldLayout *tagLayout,
		SwerveChassis *chassis, Config config) {
	this->config = config;
	this->tagLayout = tagLayout;
	this->chassis = chassis;

	camera = std::make_unique < photon::PhotonCamera
			> (this->config.cameraName);
	poseEstimator =
			std::make_unique < photon::PhotonPoseEstimator
					> (*this->tagLayout, photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR, this->config.cameraToRobot);
}
;

//Check if distance between robot and tag is less than a certain value ;)
bool AprilTags::checkTagDistance(const photon::PhotonPipelineResult &result) {
	if (result.GetTargets().size() > config.tagValidDistances.size()) {
		return true;
	}

	if (result.GetBestTarget().GetBestCameraToTarget().Translation().Distance( {
			0_m, 0_m, 0_m })
			< config.tagValidDistances.at(result.GetTargets().size())) {
		return true;
	}

	return false;
}

void AprilTags::addMeasurementToChassis(
		const photon::PhotonPipelineResult &result) {

	std::optional < photon::EstimatedRobotPose > poseResult =
			poseEstimator->Update(result);

	if (poseResult.has_value()) {
		frc::Pose2d poseTo2d = poseResult.value().estimatedPose.ToPose2d();
		chassis->addVisionMeasurement(poseTo2d, poseResult.value().timestamp);
		poseLog.Append(poseTo2d);
	}
}

//Update odometry with vision :0
void AprilTags::updateOdometry() {
	std::optional < photon::PhotonPipelineResult > result = getCameraResult();
	if (!result.has_value()) {
		return;
	}
	photon::PhotonPipelineResult pipelineResult = result.value();

	if (checkTagDistance(pipelineResult)) {
		addMeasurementToChassis(pipelineResult);
	}
}

//Get PhotonPipeResult from PhotonVision
std::optional<photon::PhotonPipelineResult> AprilTags::getCameraResult() {
	std::vector < photon::PhotonPipelineResult > results =
			camera->GetAllUnreadResults();

	if (results.empty()) {
		return std::nullopt;
	}

	return results[0];
}

void AprilTags::Periodic() {
	updateOdometry();
}

