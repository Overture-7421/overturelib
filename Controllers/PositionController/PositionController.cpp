// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "PositionController.h"

PositionController::PositionController(double kP, double kI, double kD, frc::TrapezoidProfile<units::meters>::Constraints profile)
	: frc::ProfiledPIDController<units::meters>(kP, kI, kD, profile) {
	calculatedValue = 0;
}

double PositionController::calculate(units::meter_t targetPosition, units::meter_t currentPosition) {
	calculatedValue = Calculate(currentPosition, targetPosition);

	if (AtSetpoint()) {
		calculatedValue = 0;
	}

	return calculatedValue;
}

