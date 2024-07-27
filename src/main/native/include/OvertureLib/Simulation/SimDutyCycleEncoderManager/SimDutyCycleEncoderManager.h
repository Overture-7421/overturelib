// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <map>
#include <networktables/NetworkTableInstance.h>
#include <frc/simulation/DutyCycleEncoderSim.h>

#include "OvertureLib/Sensors/OverDutyCycleEncoder/OverDutyCycleEncoder.h"

typedef std::string NTDutyCycleEncoderName;

class SimDutyCycleEncoderManager {
public:
	void AddSimDutyCycleEncoderCandidate(OverDutyCycleEncoder *encoder);
	void Init(
			const std::map<unsigned int, NTDutyCycleEncoderName> CHNToDutyCycleEncoderNameMap);
	void Update();

	static SimDutyCycleEncoderManager& GetInstance();

	SimDutyCycleEncoderManager(SimDutyCycleEncoderManager const&) = delete;
	void operator=(SimDutyCycleEncoderManager const&) = delete;
private:
	SimDutyCycleEncoderManager();

	void RegisterSimDutyCycleEncoder(OverDutyCycleEncoder *encoder);

	struct DutyCycleEncoderNTPair {
		std::shared_ptr<nt::NetworkTable> ntable;
		std::shared_ptr<frc::sim::DutyCycleEncoderSim> dutyCycleEncoder;
	};

	nt::NetworkTableInstance ntInst = nt::NetworkTableInstance::GetDefault();

	std::map<unsigned int, NTDutyCycleEncoderName> CHNToDutyCycleEncoderNameMap;
	std::map<NTDutyCycleEncoderName, DutyCycleEncoderNTPair> registeredDutyCycleEncoders;
	std::vector<OverDutyCycleEncoder*> dutyCycleEncodersToRegister;
	std::string robotName;
	bool initialized = false;
};
