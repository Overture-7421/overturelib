// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/geometry/Rotation2d.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/Trigger.h>
#include <frc2/command/button/CommandGenericHID.h>
#include <frc2/command/button/CommandXboxController.h>
#include "OvertureLib/Math/Utils.h"

#pragma once

class OverXboxController: public frc2::CommandXboxController {
public:
	OverXboxController(int port, double stickDeadzone, double triggerDeadzone);

	double getTwist();
	frc::Rotation2d getLeftStickDirection();
	frc::Rotation2d getRightStickDirection();
	frc2::Trigger leftBumperOnly();
	frc2::Trigger rightBumperOnly();
	frc2::Trigger bothBumpers();
	frc2::Trigger leftTriggerOnly();
	frc2::Trigger rightTriggerOnly();
	frc2::Trigger bothTriggers();
	frc2::Trigger leftYTrigger(double triggerTreshold);
	frc2::Trigger leftXTrigger(double triggerTreshold);
	frc2::Trigger rightYTrigger(double triggerTreshold);
	frc2::Trigger rightXTrigger(double triggerTreshold);
	frc2::Trigger rightStick(double triggerTreshold);
	frc2::Trigger leftStick(double triggerTreshold);
	frc2::CommandPtr getRumbleCommand(double intensity);

private:
	frc::Rotation2d storedLeftStickDirection;
	frc::Rotation2d storedRightStickDirection;
	double stickDeadzone;
	double triggerDeadzone;
};
