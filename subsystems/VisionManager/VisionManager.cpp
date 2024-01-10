// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "VisionManager.h"

VisionManager::VisionManager() {};
void VisionManager::setCameraAndLayout(photonlib::PhotonCamera* camera, frc::AprilTagFieldLayout* tagLayout, frc::Transform3d* cameraToRobot) {
	this->m_Camera = camera;
	this->m_TagLayout = tagLayout;
	this->m_CameraToRobot = cameraToRobot;
}

/**
 * @brief Set the pose estimator for the robot
*/
void VisionManager::initPoseEstimator() {
	poseEstimator = new photonlib::PhotonPoseEstimator{
		*m_TagLayout,
		photonlib::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
		std::move(photonlib::PhotonCamera{ "Arducam_OV9281_USB_Camera" }),
		*m_CameraToRobot
	};

	poseEstimatorSet = true;

}

/**
 * @brief Set the color of the alliance
*/
void VisionManager::setAlliancesColor() {
	std::optional<frc::DriverStation::Alliance> allianceColor = frc::DriverStation::GetAlliance();

	if (!allianceColor.has_value()) {
		return;
	}

	if (allianceColor == frc::DriverStation::Alliance::kBlue) {
		m_TagLayout->SetOrigin(frc::AprilTagFieldLayout::OriginPosition::kBlueAllianceWallRightSide);
	} else {
		m_TagLayout->SetOrigin(frc::AprilTagFieldLayout::OriginPosition::kRedAllianceWallRightSide);
	}

	initPoseEstimator();
}

/**
 * @brief Check if distance between robot and tag is less than a certain value
 *
 * @param numberOfTags Number of tags to check
 * @param distance Distance to check
 * @return True if distance is less than the value
 * @return False if distance is greater than the value
 */
bool VisionManager::checkTagDistance(size_t numberOfTags, double distance) {
	std::optional<photonlib::PhotonPipelineResult> result = getCameraResult();

	if (result.has_value()) {
		photonlib::PhotonPipelineResult resultValue = result.value();
		if (resultValue.GetTargets().size() == numberOfTags) {
			if (resultValue.GetBestTarget().GetBestCameraToTarget().X().value() < distance) {
				return true;
			}
		}
	}

	return false;
}

/**
 * @brief Add measurement to the chassis
*/
void VisionManager::addMeasurementToChassis() {
	std::optional<photonlib::EstimatedRobotPose> poseResult = poseEstimator->Update();

	if (poseResult.has_value()) {
		photonlib::EstimatedRobotPose pose = poseResult.value();
		frc::Pose2d poseTo2d = pose.estimatedPose.ToPose2d();
		swerveChassis->addVisionMeasurement({ poseTo2d.X(), poseTo2d.Y(), swerveChassis->getOdometry().Rotation() }, pose.timestamp);
	}
}

/**
 * @brief Update the odometry of the robot
*/
void VisionManager::updateOdometry() {
	if (checkTagDistance(1, 5.00) || checkTagDistance(2, 7.00) || checkTagDistance(3, 8.00)) {
		addMeasurementToChassis();
	}
}

/**
 * @brief Update the pose estimator
 *
 * @return std::optional<photonlib::EstimatedRobotPose> The estimated pose of the robot
 */
std::optional<photonlib::PhotonPipelineResult> VisionManager::getCameraResult() {
	return m_Camera->GetLatestResult();
}

/**
 * @brief Check if the pose estimator is set
 *
 * @return True if the pose estimator is set
 * @return False if the pose estimator is not set
 */
bool VisionManager::isPoseEstimatorSet() {
	return poseEstimatorSet;
}

/**
 * @brief Set the pose estimator
 *
 * @param set True if the pose estimator is set
 * @param set False if the pose estimator is not set
 */
void VisionManager::setPoseEstimator(bool set) {
	poseEstimatorSet = set;
}

void VisionManager::Periodic() {
	// frc::SmartDashboard::PutBoolean("Set Camara", isPoseEstimatorSet());
	if (isPoseEstimatorSet()) {
		updateOdometry();
	}
}

