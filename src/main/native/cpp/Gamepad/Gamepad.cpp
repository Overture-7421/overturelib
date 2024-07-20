// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "OvertureLib/Gamepad/Gamepad.h"

Gamepad::Gamepad(int port, double stickDeadzone, double triggerDeadzone) : frc2::CommandXboxController(
		port) {
	this->stickDeadzone = stickDeadzone;
	this->triggerDeadzone = triggerDeadzone;
}
;

double Gamepad::getTwist() {
	double right = GetRightTriggerAxis();
	double left = GetLeftTriggerAxis();
	double value = right - left;
	return value;
}
;

frc::Rotation2d Gamepad::getLeftStickDirection() {
	double x = Utils::ApplyAxisFilter(-GetLeftX(), stickDeadzone);
	double y = Utils::ApplyAxisFilter(-GetLeftY(), stickDeadzone);

	return frc::Rotation2d(x, y);
}
;

frc::Rotation2d Gamepad::getRightStickDirection() {
	double x = Utils::ApplyAxisFilter(-GetRightX(), stickDeadzone);
	double y = Utils::ApplyAxisFilter(-GetRightY(), stickDeadzone);

	return frc::Rotation2d(x, y);
}
;

frc2::Trigger Gamepad::noBumpers() {
	return !RightBumper() && !LeftBumper();
}

frc2::Trigger Gamepad::leftBumperOnly() {
	return LeftBumper() && !RightBumper();
}

frc2::Trigger Gamepad::rightBumperOnly() {
	return RightBumper() && !LeftBumper();
}

frc2::Trigger Gamepad::bothBumpers() {
	return RightBumper() && LeftBumper();
}

frc2::Trigger Gamepad::noTriggers() {
	return !LeftTrigger(triggerDeadzone) && !RightTrigger(triggerDeadzone);
}

frc2::Trigger Gamepad::leftTriggerOnly() {
	return LeftTrigger(triggerDeadzone) && !RightTrigger(triggerDeadzone);
}

frc2::Trigger Gamepad::rightTriggerOnly() {
	return RightTrigger(triggerDeadzone) && !LeftTrigger(triggerDeadzone);
}

frc2::Trigger Gamepad::bothTriggers() {
	return LeftTrigger(triggerDeadzone) && RightTrigger(triggerDeadzone);
}

frc2::Trigger Gamepad::upDpad() {
	return frc2::Trigger([this] {
		return GetPOV() == 0;
	});
}

frc2::Trigger Gamepad::downDpad() {
	return frc2::Trigger([this] {
		return GetPOV() == 180;
	});
}

frc2::Trigger Gamepad::leftDpad() {
	return frc2::Trigger([this] {
		return GetPOV() == 270;
	});
}

frc2::Trigger Gamepad::rightDpad() {
	return frc2::Trigger([this] {
		return GetPOV() == 90;
	});
}

frc2::CommandPtr Gamepad::rumbleCommand(double intensity) {
	return frc2::cmd::RunOnce([this, intensity] {
		SetRumble(frc::GenericHID::RumbleType::kBothRumble, intensity);
	});
}

frc2::Trigger Gamepad::leftYTrigger(double triggerTreshold) {
	return frc2::Trigger([this, triggerTreshold] {
		return std::abs(GetLeftY()) >= triggerTreshold;
	});
}

frc2::Trigger Gamepad::leftXTrigger(double triggerTreshold) {
	return frc2::Trigger([this, triggerTreshold] {
		return std::abs(GetLeftX()) >= triggerTreshold;
	});
}

frc2::Trigger Gamepad::rightYTrigger(double triggerTreshold) {
	return frc2::Trigger([this, triggerTreshold] {
		return std::abs(GetRightY()) >= triggerTreshold;
	});
}

frc2::Trigger Gamepad::rightXTrigger(double triggerTreshold) {
	return frc2::Trigger([this, triggerTreshold] {
		return std::abs(GetRightX()) >= triggerTreshold;
	});
}

frc2::Trigger Gamepad::rightStick(double triggerTreshold) {
	return frc2::Trigger(
			[this, triggerTreshold] {
				return std::abs(GetRightX()) >= triggerTreshold
						|| std::abs(GetRightY()) >= triggerTreshold;
			});
}

frc2::Trigger Gamepad::leftStick(double triggerTreshold) {
	return frc2::Trigger(
			[this, triggerTreshold] {
				return std::abs(GetLeftX()) >= triggerTreshold
						|| std::abs(GetLeftY()) >= triggerTreshold;
			});
}
