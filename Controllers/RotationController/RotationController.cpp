// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RotationController.h"

RotationController::RotationController(double kP, double kI, double kD, frc::TrapezoidProfile<units::radians>::Constraints profile)
	: frc::ProfiledPIDController<units::radians>(kP, kI, kD, profile) {
	EnableContinuousInput(units::radian_t(-180_deg), units::radian_t(180_deg));
	SetIZone(3);

	calculatedValue = 0;
}

double RotationController::calculateValue(units::radian_t targetAngle, units::radian_t currentAngle) {
	calculatedValue = Calculate(currentAngle, targetAngle);

	if (AtSetpoint()) {
		calculatedValue = 0;
	}

	return calculatedValue;
}