// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <map>
#include <wpi/array.h>
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
#include "OvertureLib/Subsystems/Vision/AprilTags/LimelightHelpers.h"

enum class VisionBackend {
	PhotonVision, Limelight
};

enum class LimelightMode {
	MegaTag1, MegaTag2
};

class AprilTags: public frc2::SubsystemBase {
public:
	struct Config {
		VisionBackend backend = VisionBackend::PhotonVision;
		std::string cameraName;
		std::function<frc::Transform3d()> cameraToRobotSupplier;

		// Limelight-specific
		LimelightMode limelightMode = LimelightMode::MegaTag2;

		std::map<int, units::meter_t> tagValidDistances = { { 1, 3.5_m }, { 2,
				6.0_m }, { 3, 8.0_m } };

		wpi::array<double, 3> singleTagStdDevs { 2.0, 2.0, 2.0 };
		wpi::array<double, 3> multiTagStdDevs { 0.07, 0.07, 0.5 };

		// Limelight MegaTag2 complementary filter alpha (mode 4).
		// Higher = LL internal IMU tracks the external (chassis) yaw faster,
		// so corrections from resetHeading / Photon propagate quickly.
		// Default LL value is 0.001; we bump it slightly for faster recovery.
		double imuAssistAlpha = 0.005;

		// MegaTag1 yaw watchdog: hard-snap chassis heading when MegaTag1
		// disagrees with the fused estimator yaw by more than this threshold
		// for `yawCorrectionMinStreak` consecutive frames, while moving
		// slower than `yawCorrectionMaxSpeed`.
		units::degree_t yawCorrectionThreshold = 5_deg;
		int yawCorrectionMinTags = 2;
		int yawCorrectionMinStreak = 5;
		units::meters_per_second_t yawCorrectionMaxSpeed = 0.5_mps;
	};

	AprilTags(frc::AprilTagFieldLayout *tagLayout, SwerveChassis *chassis,
			Config config);
	wpi::array<double, 3> GetEstimationStdDevs(int tagCount,
			units::meter_t avgDist, frc::Pose2d estimatedPose,
			bool trustRotation = true);
	void addMeasurementToChassis(frc::Pose2d pose, units::second_t timestamp,
			int tagCount, units::meter_t avgDist, bool trustRotation = true);
	void setEnabled(bool enabled);
	void Periodic() override;

private:
	void PeriodicPhotonVision();
	void PeriodicLimelight();

	/* PhotonVision */
	std::unique_ptr<photon::PhotonCamera> camera;
	std::unique_ptr<photon::PhotonPoseEstimator> poseEstimator;

	frc::AprilTagFieldLayout *tagLayout;
	SwerveChassis *chassis;
	Config config;
	bool enabled = true;
	bool lastRobotEnabled = false;
	int yawErrorStreak = 0;
	nt::StructArrayPublisher<frc::Pose3d> targetPosesPublisher;
	nt::StructPublisher<frc::Pose2d> visionPose2dPublisher;

#ifndef __FRC_ROBORIO__ // If on simulation
	std::shared_ptr<photon::PhotonCameraSim> cameraSim;
#endif
};
