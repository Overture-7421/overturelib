// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <frc/TimedRobot.h>
#include "RobotConstants.h"

#ifndef __FRC_ROBORIO__
#include "OvertureLib/Simulation/SimMotorManager/SimMotorManager.h"
#include "OvertureLib/Simulation/SimPigeonManager/SimPigeonManager.h"
#include "OvertureLib/Simulation/SimCANCoderManager/SimCANCoderManager.h"
#include "OvertureLib/Simulation/SimDutyCycleEncoderManager/SimDutyCycleEncoderManager.h"
#endif

/**
 * Implementation of TimedRobot that allows to seamlessly change between simulation (using Gazebo) and a real robot.
 */

class OverRobot: public frc::TimedRobot {
public:
	OverRobot(units::second_t period = 20_ms) : frc::TimedRobot(period) {
#ifndef __FRC_ROBORIO__
 	nt::NetworkTableInstance::GetDefault().StopServer();
	nt::NetworkTableInstance::GetDefault().StartClient4("Offseason 2024");
	nt::NetworkTableInstance::GetDefault().SetServer("127.0.0.1");
	
		AddPeriodic([&] {
			simPigeonManager.Update();
			simCANCoderManager.Update();
			simDutyCycleEncoderManager.Update();
			simMotorManager.Update();
		}, 5_ms);
#endif
	}

#ifndef __FRC_ROBORIO__
public:
	SimMotorManager &simMotorManager = SimMotorManager::GetInstance();
	SimPigeonManager &simPigeonManager = SimPigeonManager::GetInstance();
	SimCANCoderManager &simCANCoderManager = SimCANCoderManager::GetInstance();
	SimDutyCycleEncoderManager &simDutyCycleEncoderManager =
			SimDutyCycleEncoderManager::GetInstance();
#endif
};
