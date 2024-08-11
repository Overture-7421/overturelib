// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

struct ModuleConfig {
	ModuleConfig(frc::SimpleMotorFeedforward<units::meters> FeedForward) : FeedForward(
			FeedForward) {

	}
	int DrivedId = -1;
	int TurnId = -1;
	int CanCoderId = -1;

	ControllerNeutralMode DriveNeutralMode = ControllerNeutralMode::Brake;
	ControllerNeutralMode TurnNeutralMode = ControllerNeutralMode::Coast;

	bool DriveInverted = false;
	bool TurnInverted = true;

	units::turn_t Offset = 0_deg;
	std::string ModuleName = "";
	std::string CanBus = "";

	double kP = 0;
	double kI = 0;
	double kD = 0;

	frc::SimpleMotorFeedforward<units::meters> FeedForward { 0_V, 0_V / 1_mps,
			0_V / 1_mps_sq };

	double TurnGearRatio = 1.0;
	double DriveGearRatio = 1.0;

	units::meter_t WheelDiameter = 1_m;

	double DriveCurrentLimit = 90;
	double DriveStatorCurrentLimit = 110;
	double DriveTriggerThreshold = 0;
	double DriveTriggerThresholdTime = 0.5;
	double DriveRampRate = 0.25;

	double TurnCurrentLimit = 60;
	double TurnStatorCurrentLimit = 80;
	double TurnTriggerThreshold = 0;
	double TurnTriggerThresholdTime = 0.2;
	double TurnRampRate = 0.0;
};
