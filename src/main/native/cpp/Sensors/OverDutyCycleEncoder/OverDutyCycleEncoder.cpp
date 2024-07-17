// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "OvertureLib/Sensors/OverDutyCycleEncoder/OverDutyCycleEncoder.h"

#ifndef __FRC_ROBORIO__
#include "OvertureLib/Simulation/SimDutyCycleEncoderManager/SimDutyCycleEncoderManager.h"
#endif

OverDutyCycleEncoder::OverDutyCycleEncoder(int channel) : frc::DutyCycleEncoder(channel) {
#ifndef __FRC_ROBORIO__
	SimDutyCycleEncoderManager* simDutyCycleEncoderManager = SimDutyCycleEncoderManager::GetInstance();
	simDutyCycleEncoderManager->AddSimDutyCycleEncoderCandidate(this);
#endif
};
