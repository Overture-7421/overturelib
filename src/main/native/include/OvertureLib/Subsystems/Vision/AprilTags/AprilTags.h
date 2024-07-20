// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/geometry/Pose3d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/DriverStation.h>

#include "OvertureLib/Subsystems/Swerve/SwerveChassis/SwerveChassis.h"

class AprilTags: public frc2::SubsystemBase {
public:
	struct Config {
		std::string cameraName;
		frc::Transform3d cameraToRobot;
		units::meter_t singleTagValidDistance = 3.5_m;
		units::meter_t doubleTagValidDistance = 6.0_m;
		units::meter_t tripleTagValidDistance = 8.0_m;
	};

	AprilTags(frc::AprilTagFieldLayout *tagLayout, SwerveChassis *chassis,
			Config config);
	bool checkTagDistance(const photon::PhotonPipelineResult &result,
			size_t numberOfTags, units::meter_t distance);
	void addMeasurementToChassis(const photon::PhotonPipelineResult &result);
	void updateOdometry();
	std::optional<photon::EstimatedRobotPose> update(
			const photon::PhotonPipelineResult &result);
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
	wpi::log::StructLogEntry<frc::Pose2d> poseLog = wpi::log::StructLogEntry
			< frc::Pose2d > (log, "/vision/pose");
};
