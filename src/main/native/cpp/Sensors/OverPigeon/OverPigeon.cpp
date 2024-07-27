// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "OvertureLib/Sensors/OverPigeon/OverPigeon.h"

#ifndef __FRC_ROBORIO__
#include "OvertureLib/Simulation/SimPigeonManager/SimPigeonManager.h"
#endif

OverPigeon::OverPigeon(int deviceId, std::string canbus) : ctre::phoenix6::hardware::Pigeon2(
		deviceId, canbus) {
#ifndef __FRC_ROBORIO__
	SimPigeonManager &simPigeonManager = SimPigeonManager::GetInstance();
	simPigeonManager.SetSimPigeon(this);
#endif

}
