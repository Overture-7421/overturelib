// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <OvertureLib/Subsystems/Swerve/SpeedsHelper/SpeedsHelper.h>
#include <OvertureLib/Subsystems/Swerve/SwerveChassis/SwerveChassis.h>

// Class for controlling the rotation of the robot
class HeadingSpeedsHelper: public SpeedsHelper {
public:
	HeadingSpeedsHelper(
			frc::ProfiledPIDController<units::radian> headingController,
			SwerveChassis *chassis);
	void setTargetAngle(frc::Rotation2d targetAngle);
	void alterSpeed(frc::ChassisSpeeds &inputSpeed) override;

private:
	frc::ProfiledPIDController<units::radian> headingController;
	SwerveChassis *chassis = nullptr;
	units::radian_t targetAngle;
};
