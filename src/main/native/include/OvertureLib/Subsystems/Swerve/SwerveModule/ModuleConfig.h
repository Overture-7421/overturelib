// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>
#include <OvertureLib/MotorControllers/OverTalonFX/Config.h>
#include <OvertureLib/Sensors/OverCANCoder/Config.h>

static OverTalonFXConfig DriveInit() {
	OverTalonFXConfig config;

	config.NeutralMode = ControllerNeutralMode::Brake;

	config.CurrentLimit = 90_A;
	config.StatorCurrentLimit = 120_A;
	config.TriggerThreshold = 120_A;
	config.TriggerThresholdTime = 0.5_s;
	config.ClosedLoopRampRate = 0_s;
	config.OpenLoopRampRate = 0.25_s;
	return config;
}

static OverTalonFXConfig TurnInit() {
	OverTalonFXConfig config;

	config.NeutralMode = ControllerNeutralMode::Coast;

	config.CurrentLimit = 60_A;
	config.StatorCurrentLimit = 80_A;
	config.TriggerThreshold = 80_A;
	config.TriggerThresholdTime = 0.2_s;
	config.ClosedLoopRampRate = 0_s;
	config.OpenLoopRampRate = 0_s;
	return config;
}

struct SwerveModuleConfig {
	SwerveModuleConfig(frc::SimpleMotorFeedforward<units::meters> FeedForward) : FeedForward(
			FeedForward) {
	}

	OverTalonFXConfig DriveMotorConfig = DriveInit();

	OverTalonFXConfig TurnMotorConfig = TurnInit();

	CanCoderConfig EncoderConfig;

	std::string ModuleName = "";
	std::string CanBus = "";

	units::meter_t WheelDiameter = 0.1016_m;
	double TurnGearRatio = 1.0;
	double DriveGearRatio = 1.0;

	frc::SimpleMotorFeedforward<units::meters> FeedForward { 0_V, 0_V / 1_mps,
			0_V / 1_mps_sq };
};
