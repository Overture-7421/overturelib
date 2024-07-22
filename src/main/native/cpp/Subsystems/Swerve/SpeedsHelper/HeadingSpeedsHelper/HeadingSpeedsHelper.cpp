// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "OvertureLib/Subsystems/Swerve/SpeedsHelper/HeadingSpeedsHelper/HeadingSpeedsHelper.h"

HeadingSpeedsHelper::HeadingSpeedsHelper(frc::ProfiledPIDController<units::radian> headingController, SwerveChassis* chassis) : headingController(headingController){
	this->chassis = chassis;
	this->headingController.EnableContinuousInput(units::radian_t(-180_deg), units::radian_t(180_deg));
	this->headingController.SetIZone(3);
}

void HeadingSpeedsHelper::alterSpeed(frc::ChassisSpeeds &inputSpeed) {
	double out = headingController.Calculate(chassis->getRotation2d().Radians(), targetAngle);

	if (headingController.AtSetpoint()) {
		out = 0;
	}

	inputSpeed.omega = units::radians_per_second_t(out);
}
