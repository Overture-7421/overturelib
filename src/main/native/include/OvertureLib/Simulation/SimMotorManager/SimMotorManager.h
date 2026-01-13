// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

typedef std::string NTMotorName;

class OverTalonFX;

class SimMotorManager {
public:
	void AddSimMotorCandidate(OverTalonFX *motor);
	void Init(const std::map<unsigned int, NTMotorName> CANIDToMotorNameMap);
	void Update();

	static SimMotorManager& GetInstance();

	SimMotorManager(SimMotorManager const&) = delete;
	void operator=(SimMotorManager const&) = delete;
private:
	SimMotorManager();
	void RegisterSimMotor(OverTalonFX *motor);

	struct MotorNTPair {
		std::shared_ptr<nt::NetworkTable> ntable;
		OverTalonFX *motor;
	};

	nt::NetworkTableInstance ntInst = nt::NetworkTableInstance::GetDefault();

	std::map<unsigned int, NTMotorName> CANIDToMotorNameMap;
	std::map<NTMotorName, MotorNTPair> registeredMotors;
	std::vector<OverTalonFX*> motorsToRegister;
	std::string robotName;
	bool initialized = false;
};
