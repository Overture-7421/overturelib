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
	void AddSimDutyCycleEncoderCandidate(OverDutyCycleEncoder* encoder);
	void Init(const std::map<unsigned int, NTDutyCycleEncoderName> CHNToDutyCycleEncoderNameMap);
	void Update();

	static SimDutyCycleEncoderManager* GetInstance() {
		// If there is no instance of class
		// then we can create an instance.
		if (instancePtr == NULL) {
			// We can access private members 
			// within the class.
			instancePtr = new SimDutyCycleEncoderManager();

			// returning the instance pointer
			return instancePtr;
		} else {
			// if instancePtr != NULL that means 
			// the class already have an instance. 
			// So, we are returning that instance 
			// and not creating new one.
			return instancePtr;
		}
	}
	SimDutyCycleEncoderManager(const SimDutyCycleEncoderManager& obj) = delete;

private:
	SimDutyCycleEncoderManager();
	void RegisterSimDutyCycleEncoder(OverDutyCycleEncoder* encoder);

	struct DutyCycleEncoderNTPair {
		std::shared_ptr<nt::NetworkTable> ntable;
		std::shared_ptr<frc::sim::DutyCycleEncoderSim> dutyCycleEncoder;
	};

	nt::NetworkTableInstance ntInst = nt::NetworkTableInstance::GetDefault();

	std::map<unsigned int, NTDutyCycleEncoderName> CHNToDutyCycleEncoderNameMap;
	std::map<NTDutyCycleEncoderName, DutyCycleEncoderNTPair> registeredDutyCycleEncoders;
	std::vector<OverDutyCycleEncoder*> dutyCycleEncodersToRegister;
	std::string robotName;


	static SimDutyCycleEncoderManager* instancePtr;
};
