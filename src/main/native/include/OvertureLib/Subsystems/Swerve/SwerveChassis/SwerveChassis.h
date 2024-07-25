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

#include <OvertureLib/Sensors/OverPigeon/OverPigeon.h>
#include <OvertureLib/Subsystems/Swerve/SwerveModule/SwerveModule.h>
#include <OvertureLib/Subsystems/Swerve/SwerveChassis/SwerveBase.h>
#include <OvertureLib/Math/ChassisAccels.h>
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

	void resetHeading(double angle = 0);

	frc2::CommandPtr SysIdQuadstatic(frc2::sysid::Direction direction);
	frc2::CommandPtr SysIdDinamic(frc2::sysid::Direction direction);
	void sysIdVoltage(units::volt_t voltage);

	void shuffleboardPeriodic();
	void Periodic() override;

private:
	void setModuleStates(
			const wpi::array<frc::SwerveModuleState, 4> &desiredStates);
	void updateOdometry();

	frc::Pose2d latestPose;

	frc::ChassisSpeeds desiredSpeeds;
	frc::ChassisSpeeds currentSpeeds, lastSpeeds;
	ChassisAccels currentAccels;

	frc::Field2d field2d;

	std::optional<SpeedsHelper*> speedsHelper;
	bool acceptingVisionMeasurements = false;

	bool characterizing = false;frc2::sysid::SysIdRoutine m_sysIdRoutine {
		frc2::sysid::Config {std::nullopt, std::nullopt, std::nullopt, std::nullopt},
		frc2::sysid::Mechanism {
			[this](units::volt_t driveVoltage) {
				sysIdVoltage(driveVoltage);
			},
			[this](frc::sysid::SysIdRoutineLog* log) {
				log->Motor("frontRight")
				.voltage(units::volt_t {getFrontRightModule().getVoltage()})
				.position(units::meter_t {getFrontRightModule().getDistance()})
				.velocity(units::meters_per_second_t {getFrontRightModule().getSpeed()});
				log->Motor("frontLeft")
				.voltage(units::volt_t {getFrontLeftModule().getVoltage()})
				.position(units::meter_t {getFrontLeftModule().getDistance()})
				.velocity(units::meters_per_second_t {getFrontLeftModule().getSpeed()});
				log->Motor("backRight")
				.voltage(units::volt_t {getBackRightModule().getVoltage()})
				.position(units::meter_t {getBackRightModule().getDistance()})
				.velocity(units::meters_per_second_t {getBackRightModule().getSpeed()});
				log->Motor("backLeft")
				.voltage(units::volt_t {getBackLeftModule().getVoltage()})
				.position(units::meter_t {getBackLeftModule().getDistance()})
				.velocity(units::meters_per_second_t {getBackLeftModule().getSpeed()});
			},
			this}};
};
