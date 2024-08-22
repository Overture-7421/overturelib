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
	bool EncoderInverted = false;

	units::turn_t Offset = 0_deg;
	std::string ModuleName = "";
	std::string CanBus = "";

	SlotConfigs TurnPIDConfigs;

	frc::SimpleMotorFeedforward<units::meters> FeedForward { 0_V, 0_V / 1_mps,
			0_V / 1_mps_sq };

	double TurnGearRatio = 1.0;
	double DriveGearRatio = 1.0;

	units::meter_t WheelDiameter = 1_m;

	units::ampere_t DriveCurrentLimit = 90_A;
	units::ampere_t DriveStatorCurrentLimit = 110_A;
	units::ampere_t DriveTriggerThreshold = 0_A;
	units::second_t DriveTriggerThresholdTime = 0.5_s;
	units::second_t DriveRampRate = 0.25_s;

	units::ampere_t TurnCurrentLimit = 60_A;
	units::ampere_t TurnStatorCurrentLimit = 80_A;
	units::ampere_t TurnTriggerThreshold = 0_A;
	units::second_t TurnTriggerThresholdTime = 0.2_s;
	units::second_t TurnRampRate = 0.0_s;
};
