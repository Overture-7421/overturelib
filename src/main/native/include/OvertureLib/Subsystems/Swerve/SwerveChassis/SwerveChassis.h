// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <vector>
#include <frc/geometry/Pose2d.h>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc2/command/Commands.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <frc/DataLogManager.h>
#include <fmt/format.h>
#include <frc2/command/SubsystemBase.h>
// #include <pathplanner/lib/util/swerve/SwerveSetpoint.h>
// #include <pathplanner/lib/util/swerve/SwerveSetpointGenerator.h>
#include <networktables/StructTopic.h>

#include <OvertureLib/Sensors/OverPigeon/OverPigeon.h>
#include <OvertureLib/Subsystems/Swerve/SwerveChassis/SwerveBase.h>
#include <OvertureLib/Subsystems/Swerve/SpeedsHelper/SpeedsHelper.h>
#include <OvertureLib/Robots/OverRobot/RobotConstants.h>

#include <wpi/DataLog.h>

class SwerveChassis: public SwerveBase, public frc2::SubsystemBase {
public:
	SwerveChassis();
	void disableSpeedHelper();
	void enableSpeedHelper(SpeedsHelper *speedsHelper);

	void setTargetSpeeds(frc::ChassisSpeeds speeds) override;
	frc::ChassisSpeeds getCurrentSpeeds() override;
	ChassisAccels getCurrentAccels();

	const frc::Pose2d& getEstimatedPose() override;
	void resetOdometry(frc::Pose2d initPose) override;

	void addVisionMeasurement(frc::Pose2d pose, units::second_t Latency);
	void setAcceptingVisionMeasurements(bool acceptingVisionMeasurements);

	void resetHeading(units::degree_t angle = 0_deg);

	frc2::CommandPtr SysIdQuadstatic(frc2::sysid::Direction direction);
	frc2::CommandPtr SysIdDinamic(frc2::sysid::Direction direction);
	void sysIdVoltage(units::volt_t voltage);

	void shuffleboardPeriodic();
	void Periodic() override;

private:
	void setModuleStates(
			const std::vector<frc::SwerveModuleState> &desiredStates);
	void updateOdometry();

	// pathplanner::SwerveSetpointGenerator m_setpointGenerator;
	// pathplanner::SwerveSetpoint previousSetpoint;
	frc::Pose2d latestPose;

	frc::ChassisSpeeds desiredSpeeds;
	frc::ChassisSpeeds currentSpeeds, lastSpeeds;
	ChassisAccels currentAccels;

	std::optional<SpeedsHelper*> speedsHelper;
	bool acceptingVisionMeasurements = false;

	wpi::log::DataLog &log = frc::DataLogManager::GetLog();
	wpi::log::StructLogEntry<frc::Pose2d> poseLog = wpi::log::StructLogEntry
			< frc::Pose2d > (log, "/swerve/pose");

	nt::StructPublisher<frc::Pose2d> posePublisher =
			nt::NetworkTableInstance::GetDefault().GetStructTopic < frc::Pose2d
					> ("SmartDashboard/SwerveChassis/Odometry/Pose").Publish();

	bool characterizing = false;frc2::sysid::SysIdRoutine m_sysIdRoutine {
		frc2::sysid::Config {std::nullopt, std::nullopt, std::nullopt, nullptr},
		frc2::sysid::Mechanism {
			[this](units::volt_t driveVoltage) {
				sysIdVoltage(driveVoltage);
			},
			[this](frc::sysid::SysIdRoutineLog* log) {
				log->Motor("frontRight")
				.voltage(getFrontRightModule().getVoltageDrive())
				.position(getFrontRightModule().getPosition().distance)
				.velocity(getFrontRightModule().getState().speed);
				log->Motor("frontLeft")
				.voltage(getFrontLeftModule().getVoltageDrive())
				.position(getFrontLeftModule().getPosition().distance)
				.velocity(getFrontLeftModule().getState().speed);
				log->Motor("backRight")
				.voltage(getBackRightModule().getVoltageDrive())
				.position(getBackRightModule().getPosition().distance)
				.velocity(getBackRightModule().getState().speed);
				log->Motor("backLeft")
				.voltage(getBackLeftModule().getVoltageDrive())
				.position(getBackLeftModule().getPosition().distance)
				.velocity(getBackLeftModule().getState().speed);
			},
			this}};
};
