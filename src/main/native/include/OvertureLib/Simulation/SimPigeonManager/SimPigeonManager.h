// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include "OvertureLib/Sensors/OverPigeon/OverPigeon.h"

class SimPigeonManager {
public:
	void SetSimPigeon(OverPigeon *pigeon);
	void Init(std::string imuName);
	void Update();

	static SimPigeonManager* GetInstance() {
		// If there is no instance of class
		// then we can create an instance.
		if (instancePtr == NULL) {
			// We can access private members 
			// within the class.
			instancePtr = new SimPigeonManager();

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
	SimPigeonManager(const SimPigeonManager &obj) = delete;

private:
	SimPigeonManager();
	OverPigeon *pigeon = NULL;
	ctre::phoenix6::sim::Pigeon2SimState *pigeonSimState = NULL;

	nt::NetworkTableInstance ntInst = nt::NetworkTableInstance::GetDefault();
	nt::NetworkTableEntry rollEntry, pitchEntry, yawEntry;

	static SimPigeonManager *instancePtr;

};
