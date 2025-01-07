// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <map>
#include <photon/PhotonCamera.h>
#include <photon/simulation/PhotonCameraSim.h>
#include <photon/PhotonPoseEstimator.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/geometry/Pose3d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/DriverStation.h>
#include <networktables/StructArrayTopic.h>

#include "OvertureLib/Subsystems/Swerve/SwerveChassis/SwerveChassis.h"

class AprilTags: public frc2::SubsystemBase {
public:
	struct Config {
		std::string cameraName;
		frc::Transform3d cameraToRobot;

		std::map<int, units::meter_t> tagValidDistances = { { 1, 3.5_m }, { 2,
				6.0_m }, { 3, 8.0_m } };
	};

	AprilTags(frc::AprilTagFieldLayout *tagLayout, SwerveChassis *chassis,
			Config config);
	bool checkTagDistance(const photon::PhotonPipelineResult &result);
	void addMeasurementToChassis(const photon::PhotonPipelineResult &result);
	void updateOdometry();
	std::optional<photon::PhotonPipelineResult> getCameraResult();
	void Periodic() override;

private:
	/* PhotonVision */
	std::unique_ptr<photon::PhotonCamera> camera;
	std::unique_ptr<photon::PhotonPoseEstimator> poseEstimator;

	frc::AprilTagFieldLayout *tagLayout;
	SwerveChassis *chassis;
	Config config;
	wpi::log::DataLog &log = frc::DataLogManager::GetLog();
	wpi::log::StructLogEntry<frc::Pose2d> poseLog;

	nt::StructArrayPublisher<frc::Pose3d> targetPosesPublisher;
	nt::StructPublisher<frc::Pose2d> visionPose2dPublisher;

#ifndef __FRC_ROBORIO__ // If on simulation
	std::shared_ptr<photon::PhotonCameraSim> cameraSim;
#endif
};
