// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Gamepad.h"

Gamepad::Gamepad(int port, double stickDeadzone, double triggerDeadzone) : frc2::CommandXboxController(port) {
	this->stickDeadzone = stickDeadzone;
	this->triggerDeadzone = triggerDeadzone;
	m_genericHID = new frc2::CommandGenericHID(port);
};


double Gamepad::getTwist() {
	double right = GetRightTriggerAxis();
	double left = GetLeftTriggerAxis();
	double value = right - left;
	return value;
};


frc::Rotation2d Gamepad::getLeftStickDirection() {
	double x = Utils::ApplyAxisFilter(-GetLeftX(), stickDeadzone);
	double y = Utils::ApplyAxisFilter(-GetLeftY(), stickDeadzone);

	return frc::Rotation2d(x, y);
};

frc::Rotation2d Gamepad::getRightStickDirection() {
	double x = Utils::ApplyAxisFilter(-GetRightX(), stickDeadzone);
	double y = Utils::ApplyAxisFilter(-GetRightY(), stickDeadzone);

	return frc::Rotation2d(x, y);
};

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

frc2::Trigger Gamepad::a() {
	return A();
};

frc2::Trigger Gamepad::b() {
	return B();
}

frc2::Trigger Gamepad::x() {
	return X();
}

frc2::Trigger Gamepad::y() {
	return Y();
}

frc2::Trigger Gamepad::start() {
	return Start();
}

frc2::Trigger Gamepad::select() {
	return Back();
}

frc2::Trigger Gamepad::upDpad() {
	return m_genericHID->POVUp();
}

frc2::Trigger Gamepad::downDpad() {
	return m_genericHID->POVDown();
}

frc2::Trigger Gamepad::leftDpad() {
	return m_genericHID->POVLeft();
}

frc2::Trigger Gamepad::rightDpad() {
	return m_genericHID->POVRight();
}

frc2::CommandPtr Gamepad::rumbleCommand(double intensity) {
	SetRumble(frc::GenericHID::RumbleType::kBothRumble, intensity);
}

frc2::Trigger Gamepad::leftYTrigger(double triggerTreshold) {
	return frc2::Trigger([this, triggerTreshold] {return std::abs(GetLeftY()) >= triggerTreshold;});
}

frc2::Trigger Gamepad::leftXTrigger(double triggerTreshold) {
	return frc2::Trigger([this, triggerTreshold] {return std::abs(GetLeftX()) >= triggerTreshold;});
}

frc2::Trigger Gamepad::rightYTrigger(double triggerTreshold) {
	return frc2::Trigger([this, triggerTreshold] {return std::abs(GetRightY()) >= triggerTreshold;});
}

frc2::Trigger Gamepad::rightXTrigger(double triggerTreshold) {

}

frc2::Trigger Gamepad::rightStick(double triggerTreshold) {
	{ [this, triggerTreshold] {return std::abs(GetRightX()) >= triggerTreshold || GetRightX() >= triggerTreshold;}; };

}

frc2::Trigger Gamepad::leftStick(double triggerTreshold) {
	{ [this, triggerTreshold] {return std::abs(GetLeftX()) >= triggerTreshold || GetLeftX() >= triggerTreshold;}; };
}