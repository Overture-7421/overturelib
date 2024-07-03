// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>

// Class for controlling the rotation of the robot
class RotationController {
public:
	RotationController(double kP, double kI, double kD, frc::TrapezoidProfile<units::radians>::Constraints profile);
	double calculate(units::radian_t targetAngle, units::radian_t currentAngle);
	bool atSetpoint();
	bool atFeedbackSetpoint();
	void reset(units::radian_t currentAngle);
	void updatePID(double kP, double kI, double kD);

private:
	// PID controller for the rotation
	frc::ProfiledPIDController<units::radians> m_controller;


	double calculatedValue = 0;

	double feedbackSetpoint;
	units::radian_t tolerance{ M_PI / 180 }; // 1 degree tolerance
};
