// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include "../SwerveChassis/SwerveChassis.h"

// Class for controlling the position of the robot
class PositionController {
public:
	PositionController(SwerveChassis& chassis, double kP, double kI, double kD, frc::TrapezoidProfile<units::meters>::Constraints profile);

	double calculate(units::meter_t targetPosition);
	bool atSetpoint();
	void reset();

private:
	SwerveChassis* m_chassis;
	// PID controller for the rotation
	frc::ProfiledPIDController<units::meters> m_controller;

	units::meter_t tolerance{ 0.04 }; // 4 cm tolerance
};
