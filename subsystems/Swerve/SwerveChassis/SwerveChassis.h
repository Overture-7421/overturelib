// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/filter/SlewRateLimiter.h>

#include <frc2/command/SubsystemBase.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
#include <pathplanner/lib/util/PIDConstants.h>
#include <pathplanner/lib/util/ReplanningConfig.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>

#include "Sensors/OverPigeon/OverPigeon.h"
#include "Controllers/RotationController/RotationController.h"
#include "Subsystems/Swerve/SwerveModule/SwerveModule.h"
#include "Math/ChassisAccels.h"

#include "Robots/OverRobot/RobotConstants.h"

#include <wpi/DataLog.h>
#include <frc/DataLogManager.h>

using namespace pathplanner;

class SwerveChassis : public frc2::SubsystemBase {
public:
	SwerveChassis(units::meters_per_second_t maxModuleSpeed, units::meter_t driveBaseRadius);
	void setTargetHeading(frc::Rotation2d rotationTarget);
	void setHeadingOverride(bool headingOverride);
	void setVyOverride(bool vyOverride);
	void setVyTarget(units::meters_per_second_t vy);
	void setModulePositions(std::array<frc::Translation2d, 4>* positions);
	void setModulesRatios(double turnRatio, double driveRatio, double wheelDiameter);
	void setModules(SwerveModule* frontLeft, SwerveModule* frontRight, SwerveModule* backleft, SwerveModule* backRight);
	void setRotatorPID(double kP, double kI, double kD);
	void setFeedForward(units::volt_t kS, units::volt_t kV, units::volt_t kA);

	void driveRobotRelative(frc::ChassisSpeeds speeds);
	void driveFieldRelative(frc::ChassisSpeeds speeds);
	frc::ChassisSpeeds getRobotRelativeSpeeds();
	frc::ChassisSpeeds getFieldRelativeSpeeds();
	ChassisAccels getFieldRelativeAccels();

	void setAlliance();
	frc::ChassisSpeeds limitSpeeds(frc::ChassisSpeeds speeds, bool isFieldRelative);
	void setDrive(frc::ChassisSpeeds speeds, bool isFieldRelative, bool isOpenLoop = true, frc::Rotation2d heading = frc::Rotation2d());

	const frc::Pose2d& getOdometry();
	void resetOdometry(frc::Pose2d initPose);
	const frc::SwerveDriveKinematics<4>& getKinematics();
	void addVisionMeasurement(frc::Pose2d pose, units::second_t Latency);
	void setAcceptingVisionMeasurements(bool acceptVisionMeasurements);
	void resetAngle(double angle = 0);

	void setModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates);
	wpi::array<frc::SwerveModuleState, 4> getModuleStates();
	wpi::array<frc::SwerveModulePosition, 4> getModulePosition();

	double getPitch();
	double getYaw();
	double getRoll();

	void updateOdometry();
	void shuffleboardPeriodic();
	void Periodic() override;

protected:
	OverPigeon* pigeon;

	SwerveModule* frontLeftModule;
	SwerveModule* frontRightModule;
	SwerveModule* backLeftModule;
	SwerveModule* backRightModule;

	std::unique_ptr<frc::SwerveDriveKinematics<4>> kinematics;

	std::array<frc::SwerveModulePosition, 4> odometryPos;

	std::unique_ptr<frc::SwerveDrivePoseEstimator<4>> odometry;
	frc::Pose2d latestPose;

	frc::ChassisSpeeds desiredSpeeds;

	frc::ChassisSpeeds fieldRelativeSpeed, lastFieldRelativeSpeed;
	ChassisAccels fieldRelativeAccel;

private:
	std::optional<frc::Rotation2d> getRotationTargetOverride();
	units::meters_per_second_t maxModuleSpeed;
	units::meter_t driveBaseRadius;

	frc::Field2d field2d;

	wpi::log::DataLog& log = frc::DataLogManager::GetLog();
	wpi::log::StructLogEntry<frc::Pose2d> poseLog = wpi::log::StructLogEntry<frc::Pose2d>(log, "/swerve/pose");
	wpi::log::StructLogEntry<frc::Pose2d> visionPoseLog = wpi::log::StructLogEntry<frc::Pose2d>(log, "/swerve/vision_pose");

	bool headingOverride = false;
	// frc::ProfiledPIDController<units::radians> headingController{ 11.0, 0.5, 0.35, frc::TrapezoidProfile<units::radians>::Constraints{ 18_rad_per_s, 18_rad_per_s_sq * 2 }, RobotConstants::LoopTime };
	RotationController headingController{ 11.0, 0.5, 0.35, {18_rad_per_s, 18_rad_per_s_sq * 2} };
	frc::Rotation2d headingTarget;

	bool vyOverride = false;
	units::meters_per_second_t vyTarget;

	bool acceptVisionMeasurements = true;

	/* Limiters */
	frc::SlewRateLimiter<units::meters_per_second> xLimiter{ 15.0_mps_sq };
	frc::SlewRateLimiter<units::meters_per_second> yLimiter{ 15.0_mps_sq };
	frc::SlewRateLimiter<units::radians_per_second> rLimiter{ 18_tr_per_s_sq };

	int alliance = 1;
};