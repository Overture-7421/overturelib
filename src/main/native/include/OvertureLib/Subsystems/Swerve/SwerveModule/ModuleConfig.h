// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>

struct ModuleConfig {
	int rotatorId;
	int wheelId;
	int canCoderId;
	units::turn_t offset;
	std::string moduleName;
	std::string canBus;

	double kP;
	double kI;
	double kD;

	units::volt_t ks;
	units::volt_t kv;
	units::volt_t ka;

	double turnGearRatio;
	double wheelGearRatio;

	double wheelDiameter;

};
