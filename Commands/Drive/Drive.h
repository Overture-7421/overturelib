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

#include "OvertureLib/subsystems/Swerve/SwerveChassis/SwerveChassis.h"
#include "OvertureLib/math/Utils.h"

class Drive
	: public frc2::CommandHelper<frc2::Command, Drive> {
public:
	Drive(SwerveChassis* swerveChassis, frc::XboxController* controller);

	void Initialize() override;

	void Execute() override;

	void End(bool interrupted) override;

	bool IsFinished() override;

private:
	SwerveChassis* m_swerveChassis;

	/* Speed Constants */
	double kMaxSpeed{ 5.75 };
	double kMaxAngularSpeed{ 9.0 };

	/* Constant */
	units::meters_per_second_squared_t maxAcceleration{ 12 };
	units::radians_per_second_squared_t maxRotation{ 16 };

	/* Limiters */
	frc::SlewRateLimiter<units::meters_per_second> xLimiter{ maxAcceleration };
	frc::SlewRateLimiter<units::meters_per_second> yLimiter{ maxAcceleration };
	frc::SlewRateLimiter<units::radians_per_second> rLimiter{ maxRotation };

	/* Controller */
	frc::XboxController* joystick;
};