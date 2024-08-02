// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "OvertureLib/Subsystems/Swerve/SpeedsHelper/HeadingSpeedsHelper/HeadingSpeedsHelper.h"

HeadingSpeedsHelper::HeadingSpeedsHelper(
		frc::ProfiledPIDController<units::radian> headingController,
		SwerveChassis *chassis) : headingController(headingController) {
	this->chassis = chassis;
	this->headingController.EnableContinuousInput(-180_deg, 180_deg);
	this->headingController.SetIZone(3);
	this->headingController.SetTolerance(1_deg);
}

void HeadingSpeedsHelper::setTargetAngle(frc::Rotation2d targetAngle) {
	this->targetAngle = targetAngle.Radians();
}

void HeadingSpeedsHelper::alterSpeed(frc::ChassisSpeeds &inputSpeed) {
	double out = headingController.Calculate(
			chassis->getEstimatedPose().Rotation().Radians(), targetAngle);

	if (headingController.AtSetpoint()) {
		out = 0;
	}

	inputSpeed.omega = units::radians_per_second_t(out);
}

void HeadingSpeedsHelper::initialize() {
	headingController.Reset(chassis->getEstimatedPose().Rotation().Radians());
}
