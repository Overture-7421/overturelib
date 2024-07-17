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
#include <frc2/command/Commands.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <frc/DataLogManager.h>
#include <fmt/format.h>
#include <frc2/command/SubsystemBase.h>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
#include <pathplanner/lib/util/PIDConstants.h>
#include <pathplanner/lib/util/ReplanningConfig.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>

#include "OvertureLib/Sensors/OverPigeon/OverPigeon.h"
#include "OvertureLib/Controllers/RotationController/RotationController.h"
#include "OvertureLib/Subsystems/Swerve/SwerveModule/SwerveModule.h"
#include "OvertureLib/Math/ChassisAccels.h"

#include "OvertureLib/Robots/OverRobot/RobotConstants.h"

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
	void setPositionAssist(bool positionAssist);
	void setPositionTarget(units::meters_per_second_t vy, units::meters_per_second_t vx);
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
	void setFieldRelative(units::meters_per_second_t x, units::meters_per_second_t y, units::radians_per_second_t r);
	void setRotationClosedLoop(units::meters_per_second_t x, units::meters_per_second_t y, frc::Rotation2d heading);
	void setRobotRelative(units::meters_per_second_t x, units::meters_per_second_t y, units::radians_per_second_t r);

	const frc::Pose2d& getOdometry();
	void resetOdometry(frc::Pose2d initPose);
	const frc::SwerveDriveKinematics<4>& getKinematics();
	void addVisionMeasurement(frc::Pose2d pose, units::second_t Latency);
	void setAcceptingVisionMeasurements(bool acceptVisionMeasurements);
	void resetAngle(double angle = 0);
	frc2::CommandPtr resetHeading();

	void setModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates);
	wpi::array<frc::SwerveModuleState, 4> getModuleStates();
	wpi::array<frc::SwerveModulePosition, 4> getModulePosition();

	double getPitch();
	double getYaw();
	double getRoll();

	frc2::CommandPtr SysIdQuadstatic(frc2::sysid::Direction direction);
	frc2::CommandPtr SysIdDinamic(frc2::sysid::Direction direction);
	void sysIdVoltage(units::volt_t voltage);

	void updateOdometry();
	void shuffleboardPeriodic();
	void Periodic() override;

	RotationController headingController{ 11.0, 0.5, 0.35, {18_rad_per_s, 18_rad_per_s_sq * 2} };

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
	frc::Rotation2d headingTarget;

	bool vyOverride = false;
	bool positionAssist = false;

	units::meters_per_second_t vyTarget;
	units::meters_per_second_t vxTarget;

	bool acceptVisionMeasurements = true;

	/* Limiters */
	frc::SlewRateLimiter<units::meters_per_second> xLimiter{ 15.0_mps_sq };
	frc::SlewRateLimiter<units::meters_per_second> yLimiter{ 15.0_mps_sq };
	frc::SlewRateLimiter<units::radians_per_second> rLimiter{ 18_tr_per_s_sq };

	int alliance = 1;

	frc2::sysid::SysIdRoutine m_sysIdRoutine{
		  frc2::sysid::Config{std::nullopt, std::nullopt, std::nullopt, std::nullopt},
		  frc2::sysid::Mechanism{
			  [this](units::volt_t driveVoltage) {
				sysIdVoltage(driveVoltage);
			  },
			  [this](frc::sysid::SysIdRoutineLog* log) {
				log->Motor("frontRight")
					.voltage(units::volt_t{frontRightModule->getVoltage()})
					.position(units::meter_t{frontRightModule->getDistance()})
					.velocity(units::meters_per_second_t{frontRightModule->getSpeed()});

				// log->Motor("frontLeft")
				// 	.voltage(units::volt_t{frontLeftModule->getVoltage()})
				// 	.position(units::meter_t{frontLeftModule->getDistance()})
				// 	.velocity(units::meters_per_second_t{frontLeftModule->getSpeed()});
				// log->Motor("backRight")
				// 	.voltage(units::volt_t{backRightModule->getVoltage()})
				// 	.position(units::meter_t{backRightModule->getDistance()})
				// 	.velocity(units::meters_per_second_t{backRightModule->getSpeed()});
				// log->Motor("backLeft")
				// 	.voltage(units::volt_t{backLeftModule->getVoltage()})
				// 	.position(units::meter_t{backLeftModule->getDistance()})
				// 	.velocity(units::meters_per_second_t{backLeftModule->getSpeed()});
			  },
			  this} };
};