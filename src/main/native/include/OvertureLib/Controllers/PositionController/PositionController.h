// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/length.h>

// Class for controlling the position of the robot
class PositionController : public frc::ProfiledPIDController<units::meters> {
public:
	PositionController(double kP, double kI, double kD, frc::TrapezoidProfile<units::meters>::Constraints profile);

	double calculate(units::meter_t targetPosition, units::meter_t currentPosition);

private:

	double calculatedValue = 0;
};
