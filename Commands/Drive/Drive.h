// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/filter/SlewRateLimiter.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>

#include "Subsystems/Swerve/SwerveChassis/SwerveChassis.h"
#include "Math/Utils.h"

class Drive
	: public frc2::CommandHelper<frc2::Command, Drive> {
public:
	Drive(units::meters_per_second_t maxModuleSpeed, SwerveChassis* swerveChassis, frc::XboxController* controller);

	void Initialize() override;

	void Execute() override;

	void End(bool interrupted) override;

	bool IsFinished() override;

private:
	SwerveChassis* m_swerveChassis;

	/* Speed Constants */
	units::meters_per_second_t kMaxModuleSpeed;
	units::radians_per_second_t kMaxAngularSpeed{ 1.5_tps };

	int alliance = 1;

	/* Limiters */
	frc::SlewRateLimiter<units::meters_per_second> xLimiter{ 15.0_mps_sq };
	frc::SlewRateLimiter<units::meters_per_second> yLimiter{ 15.0_mps_sq };
	frc::SlewRateLimiter<units::radians_per_second> rLimiter{ 18_tr_per_s_sq };

	/* PID Controllers */
	frc::ProfiledPIDController<units::radians> headingController{ 8.0, 0, 0, frc::TrapezoidProfile<units::radians>::Constraints{ 10_rad_per_s, 4_rad_per_s_sq }, RobotConstants::LoopTime };

	/* Controller */
	frc::XboxController* joystick;
};