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
		wpi::array<double, 3> singleTagStdDevs;
		wpi::array<double, 3> multiTagStdDevs;
	};

	AprilTags(frc::AprilTagFieldLayout *tagLayout, SwerveChassis *chassis,
			Config config);
	wpi::array<double, 3> GetEstimationStdDevs(
			const photon::PhotonPipelineResult &result,
			frc::Pose2d estimatedPose);
	const photon::PhotonPipelineResult& GetLatestResult() const {
		return m_latestResult;
	}
	void addMeasurementToChassis(const photon::PhotonPipelineResult &result,
			frc::Pose2d pose, units::second_t timestamp);
	void setEnabled(bool enabled);
	virtual std::optional<photon::EstimatedRobotPose> getEstimatedPose(const photon::PhotonPipelineResult& result);
	void Periodic() override;
protected:
	std::unique_ptr<photon::PhotonPoseEstimator> poseEstimator;
private:
	/* PhotonVision */
	std::unique_ptr<photon::PhotonCamera> camera;
	photon::PhotonPipelineResult m_latestResult;

	// Standard deviations for vision measurements
	wpi::array<double, 3> singleTagStdDevs { 2.0, 2.0, 4.0 };
	wpi::array<double, 3> multiTagStdDevs { 0.1, 0.1, 0.3 };

	frc::AprilTagFieldLayout *tagLayout;
	SwerveChassis *chassis;
	Config config;
	bool enabled = true;
	nt::StructArrayPublisher<frc::Pose3d> targetPosesPublisher;
	nt::StructPublisher<frc::Pose2d> visionPose2dPublisher;

#ifndef __FRC_ROBORIO__ // If on simulation
	std::shared_ptr<photon::PhotonCameraSim> cameraSim;
#endif
};
