// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "OvertureLib/Sensors/OverCANCoder/OverCANCoder.h"
#include <map>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

typedef std::string NTCANCoderName;

class SimCANCoderManager {
public:
	void AddSimCANCoderCandidate(OverCANCoder *motor);
	void Init(
			const std::map<unsigned int, NTCANCoderName> CANIDToCANCoderNameMap);
	void Update();

	static SimCANCoderManager& GetInstance();

	SimCANCoderManager(SimCANCoderManager const&) = delete;
	void operator=(SimCANCoderManager const&) = delete;
private:
	SimCANCoderManager();

	void RegisterSimCANCoder(OverCANCoder *canCoder);

	struct CANCoderNTPair {
		std::shared_ptr<nt::NetworkTable> ntable;
		OverCANCoder *canCoder;
	};

	nt::NetworkTableInstance ntInst = nt::NetworkTableInstance::GetDefault();

	std::map<unsigned int, NTCANCoderName> CANIDToCANCoderNameMap;
	std::map<NTCANCoderName, CANCoderNTPair> registeredCANCoders;
	std::vector<OverCANCoder*> canCodersToRegister;
	std::string robotName;
	bool initialized = false;
};
