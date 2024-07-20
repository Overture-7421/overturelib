// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h"
#include <map>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

typedef std::string NTMotorName;

class SimMotorManager {
public:
	void AddSimMotorCandidate(OverTalonFX *motor);
	void Init(const std::map<unsigned int, NTMotorName> CANIDToMotorNameMap);
	void Update();

	static SimMotorManager* GetInstance() {
		// If there is no instance of class
		// then we can create an instance.
		if (instancePtr == NULL) {
			// We can access private members 
			// within the class.
			instancePtr = new SimMotorManager();

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
	SimMotorManager(const SimMotorManager &obj) = delete;

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

	static SimMotorManager *instancePtr;

};
