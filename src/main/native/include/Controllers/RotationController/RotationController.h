// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>

// Class for controlling the rotation of the robot
class RotationController : public frc::ProfiledPIDController<units::radians> {
public:
	RotationController(double kP, double kI, double kD, frc::TrapezoidProfile<units::radians>::Constraints profile);
	double calculateValue(units::radian_t targetAngle, units::radian_t currentAngle);

private:
	double calculatedValue = 0;
};
