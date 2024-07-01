// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "PositionController.h"

PositionController::PositionController(SwerveChassis& chassis, double kP, double kI, double kD, frc::TrapezoidProfile<units::meters>::Constraints profile)
	: m_chassis(&chassis), m_controller(kP, kI, kD, profile) {
	m_controller.SetTolerance(tolerance);
}

double PositionController::calculate(units::meter_t targetPosition) {
	units::meter_t currentPosition = m_chassis->getOdometry().Translation().X();
	double calculatedValue = m_controller.Calculate(currentPosition, targetPosition);

	if (m_controller.AtSetpoint()) {
		calculatedValue = 0;
	}

	return calculatedValue;
}

bool PositionController::atSetpoint() {
	return m_controller.AtSetpoint();
}

void PositionController::reset() {
	m_controller.Reset(m_chassis->getOdometry().Translation().X());
}
