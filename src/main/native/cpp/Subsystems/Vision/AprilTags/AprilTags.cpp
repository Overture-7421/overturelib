// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "AprilTags.h"
#include <iostream>

AprilTags::AprilTags(frc::AprilTagFieldLayout* tagLayout, SwerveChassis* chassis, Config config) {
	this->config = config;
	this->tagLayout = tagLayout;
	this->chassis = chassis;
	
	camera = std::make_unique<photon::PhotonCamera>(this->config.cameraName);
	poseEstimator = std::make_unique<photon::PhotonPoseEstimator> (
		*this->tagLayout,
		photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
		this->config.cameraToRobot
	);
};

//Check if distance between robot and tag is less than a certain value ;)
bool AprilTags::checkTagDistance(const photon::PhotonPipelineResult& result, size_t numberOfTags, units::meter_t distance) {
	if(numberOfTags >= 4) {
		return true;
	}

	if (result.GetTargets().size() == numberOfTags) {
		if (result.GetBestTarget().GetBestCameraToTarget().Translation().Distance({0_m, 0_m, 0_m}) < distance) {
			return true;
		}
	}

	return false;
}

void AprilTags::addMeasurementToChassis(const photon::PhotonPipelineResult& result) {

	std::optional<photon::EstimatedRobotPose> poseResult = update(result);

	if (poseResult.has_value()) {
		frc::Pose2d poseTo2d = poseResult.value().estimatedPose.ToPose2d();
		chassis->addVisionMeasurement(poseTo2d, poseResult.value().timestamp);
		poseLog.Append(poseTo2d);
	}
}

//Update odometry with vision :0
void AprilTags::updateOdometry() {
	std::optional<photon::PhotonPipelineResult> result = getCameraResult();
	if(!result.has_value()) {
		return;
	}
	photon::PhotonPipelineResult pipelineResult = result.value();

	if (checkTagDistance(pipelineResult, 1, config.singleTagValidDistance) || checkTagDistance(pipelineResult, 2, config.doubleTagValidDistance) || checkTagDistance(pipelineResult, 3, config.tripleTagValidDistance)) {
		addMeasurementToChassis(pipelineResult);
	}
}

//Get EstimatedRobotPose from PhotonVision
std::optional<photon::EstimatedRobotPose> AprilTags::update(const photon::PhotonPipelineResult& result) {
	return poseEstimator->Update(result);
}

//Get PhotonPipeResult from PhotonVision
std::optional<photon::PhotonPipelineResult> AprilTags::getCameraResult() {
	return camera->GetLatestResult();
}

void AprilTags::Periodic() {
	updateOdometry();
}


