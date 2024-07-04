// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RotationController.h"

RotationController::RotationController(double kP, double kI, double kD, frc::TrapezoidProfile<units::radians>::Constraints profile)
	: m_controller(kP, kI, kD, profile) {
	m_controller.EnableContinuousInput(units::radian_t(-180_deg), units::radian_t(180_deg));
	m_controller.SetTolerance(tolerance);
	m_controller.SetIZone(3);


	calculatedValue = 0;
	feedbackSetpoint = 0.35;
}

double RotationController::calculate(units::radian_t targetAngle, units::radian_t currentAngle) {
	calculatedValue = m_controller.Calculate(currentAngle, targetAngle);

	if (atSetpoint()) {
		calculatedValue = 0;
	}

	return calculatedValue;
}

bool RotationController::atSetpoint() {
	return m_controller.AtSetpoint();
}

bool RotationController::atFeedbackSetpoint() {
	return std::abs(calculatedValue) <= feedbackSetpoint;
}

void RotationController::reset(units::radian_t currentAngle) {
	m_controller.Reset(currentAngle);
}

void RotationController::updatePID(double kP, double kI, double kD) {
	m_controller.SetPID(kP, kI, kD);
}
