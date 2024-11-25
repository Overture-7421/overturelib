// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "OvertureLib/Gamepads/OverXboxController/OverXboxController.h"

OverXboxController::OverXboxController(int port, double stickDeadzone,
		double triggerDeadzone) : frc2::CommandXboxController(port) {
	this->stickDeadzone = stickDeadzone;
	this->triggerDeadzone = triggerDeadzone;
}

double OverXboxController::getTwist() {
	// Takes into account counter-clockwise twist as positive and clockwise twist as negative
	double right = GetRightTriggerAxis();
	double left = GetLeftTriggerAxis();
	double value = left - right;
	return value;
}

frc::Rotation2d OverXboxController::getLeftStickDirection() {
	double x = Utils::ApplyAxisFilter(-GetLeftY(), stickDeadzone);
	double y = Utils::ApplyAxisFilter(-GetLeftX(), stickDeadzone);

	return frc::Rotation2d(x, y);
}

frc::Rotation2d OverXboxController::getRightStickDirection() {
	double x = Utils::ApplyAxisFilter(-GetRightY(), stickDeadzone);
	double y = Utils::ApplyAxisFilter(-GetRightX(), stickDeadzone);

	return frc::Rotation2d(x, y);
}

frc2::Trigger OverXboxController::leftBumperOnly() {
	return LeftBumper() && !RightBumper();
}

frc2::Trigger OverXboxController::rightBumperOnly() {
	return RightBumper() && !LeftBumper();
}

frc2::Trigger OverXboxController::bothBumpers() {
	return RightBumper() && LeftBumper();
}

frc2::Trigger OverXboxController::leftTriggerOnly() {
	return LeftTrigger(triggerDeadzone) && !RightTrigger(triggerDeadzone);
}

frc2::Trigger OverXboxController::rightTriggerOnly() {
	return RightTrigger(triggerDeadzone) && !LeftTrigger(triggerDeadzone);
}

frc2::Trigger OverXboxController::bothTriggers() {
	return LeftTrigger(triggerDeadzone) && RightTrigger(triggerDeadzone);
}

frc2::CommandPtr OverXboxController::getRumbleCommand(double intensity) {
	return frc2::cmd::RunOnce([this, intensity] {
		SetRumble(frc::GenericHID::RumbleType::kBothRumble, intensity);
	});
}

frc2::Trigger OverXboxController::leftYTrigger(double triggerTreshold) {
	return frc2::Trigger([this, triggerTreshold] {
		return std::abs(GetLeftY()) >= triggerTreshold;
	});
}

frc2::Trigger OverXboxController::leftXTrigger(double triggerTreshold) {
	return frc2::Trigger([this, triggerTreshold] {
		return std::abs(GetLeftX()) >= triggerTreshold;
	});
}

frc2::Trigger OverXboxController::rightYTrigger(double triggerTreshold) {
	return frc2::Trigger([this, triggerTreshold] {
		return std::abs(GetRightY()) >= triggerTreshold;
	});
}

frc2::Trigger OverXboxController::rightXTrigger(double triggerTreshold) {
	return frc2::Trigger([this, triggerTreshold] {
		return std::abs(GetRightX()) >= triggerTreshold;
	});
}

frc2::Trigger OverXboxController::rightStick(double triggerTreshold) {
	return frc2::Trigger(
			[this, triggerTreshold] {
				return std::abs(GetRightX()) >= triggerTreshold
						|| std::abs(GetRightY()) >= triggerTreshold;
			});
}

frc2::Trigger OverXboxController::leftStick(double triggerTreshold) {
	return frc2::Trigger(
			[this, triggerTreshold] {
				return std::abs(GetLeftX()) >= triggerTreshold
						|| std::abs(GetLeftY()) >= triggerTreshold;
			});
}
