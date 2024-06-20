// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Drive.h"
#include <iostream>

Drive::Drive(units::meters_per_second_t maxModuleSpeed, SwerveChassis* swerveChassis, frc::XboxController* controller) :
	m_swerveChassis(swerveChassis), joystick(controller) {
	kMaxModuleSpeed = maxModuleSpeed;
	AddRequirements(m_swerveChassis);
}

// Called when the command is initially scheduled.
void Drive::Initialize() {
	if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) {
		alliance = -1;
	} else {
		alliance = 1;
	}
}

// Called repeatedly when this Command is scheduled to run
void Drive::Execute() {
	units::meters_per_second_t xInput{ Utils::ApplyAxisFilter(-joystick->GetLeftY()) * kMaxModuleSpeed };
	units::meters_per_second_t yInput{ Utils::ApplyAxisFilter(-joystick->GetLeftX()) * kMaxModuleSpeed };
	units::radians_per_second_t rInput = 0_rad_per_s;

	units::degree_t targetHeading = 0_deg;

	if (joystick->GetRightBumper()) {
		rInput = Utils::ApplyAxisFilter(-joystick->GetRightX()) * kMaxAngularSpeed;
	} else {
		if (std::abs(-joystick->GetRightX()) < 0.1 && std::abs(-joystick->GetRightY()) < 0.1) {
			targetHeading = m_swerveChassis->getOdometry().Rotation().Degrees();
		} else {
			targetHeading = units::degree_t(Utils::CalculateTargetHeading(-joystick->GetRightX(), -joystick->GetRightY()) * 180 / M_PI);
		}

		headingController.SetGoal(frc::Rotation2d(targetHeading).Radians());

		double outOmega = headingController.Calculate(m_swerveChassis->getOdometry().Rotation().Radians());
		if (std::abs(outOmega) < 0.1 && std::abs(-joystick->GetRightX()) < 0.1 && std::abs(-joystick->GetRightY()) < 0.1) {
			outOmega = 0;
		}

		rInput = units::radians_per_second_t(outOmega);
	}


	if (joystick->GetLeftBumper()) {
		m_swerveChassis->driveRobotRelative({
			xLimiter.Calculate(xInput),
			yLimiter.Calculate(yInput),
			rLimiter.Calculate(rInput) });
	} else {
		m_swerveChassis->driveFieldRelative({
			xLimiter.Calculate(xInput * alliance),
			yLimiter.Calculate(yInput * alliance),
			rLimiter.Calculate(rInput) });
	}
}

// Called once the command ends or is interrupted.
void Drive::End(bool interrupted) {}

// Returns true when the command should end.
bool Drive::IsFinished() {
	return false;
};