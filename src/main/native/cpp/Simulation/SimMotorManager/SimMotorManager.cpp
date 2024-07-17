// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "OvertureLib/Simulation/SimMotorManager/SimMotorManager.h"
#include <iostream>
#include <frc/RobotController.h>

SimMotorManager* SimMotorManager::instancePtr = NULL;


SimMotorManager::SimMotorManager() {}

void SimMotorManager::Init(const std::map<unsigned int, NTMotorName> CANIDToMotorNameMap) {
	this->robotName = robotName;
	this->CANIDToMotorNameMap = CANIDToMotorNameMap;

	std::for_each(motorsToRegister.begin(), motorsToRegister.end(), std::bind(&SimMotorManager::RegisterSimMotor, this, std::placeholders::_1));
}

void SimMotorManager::AddSimMotorCandidate(OverTalonFX* motor) {
	motorsToRegister.emplace_back(motor);
}


void SimMotorManager::RegisterSimMotor(OverTalonFX* motor) {
	if (motor == NULL || motor == 0) {
		throw std::invalid_argument("SimMotorManager given null pointer!");
	}


	if (!this->CANIDToMotorNameMap.contains(motor->GetDeviceID())) {
		std::cout << "SimMotorManager Warning: Tried to register a motor for simulation(" << motor->GetDeviceID() << ") that is not in the given map" << std::endl;
		return;
	}

	std::string motorName = this->CANIDToMotorNameMap[motor->GetDeviceID()];


	if (this->registeredMotors.contains(motorName)) {
		std::cout << "SimMotorManager Warning: Tried to register a motor for simulation that was already registered" << std::endl;
		return;
	}

	std::shared_ptr<nt::NetworkTable> ntable = ntInst.GetTable(motorName);

	MotorNTPair newMotorPair;
	newMotorPair.motor = motor;
	newMotorPair.ntable = ntable;

	this->registeredMotors[motorName] = std::move(newMotorPair);
}

void SimMotorManager::Update() {
	for (auto motorIterator = this->registeredMotors.begin(); motorIterator != this->registeredMotors.end(); motorIterator++) {
		MotorNTPair pair = motorIterator->second;

		std::shared_ptr<nt::NetworkTable> ntable = pair.ntable;
		OverTalonFX* motor = pair.motor;

		ctre::phoenix6::sim::TalonFXSimState& simState = motor->GetSimState();

		simState.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

		bool inverted = motor->getConfig().MotorOutput.Inverted.value;

		ntable->GetEntry("software_inverted").SetBoolean(inverted);

		units::volt_t motorVoltage = simState.GetMotorVoltage();
		ntable->GetEntry("voltage_applied").SetDouble(motorVoltage.value());

		units::turn_t motorPosition = units::turn_t(ntable->GetEntry("encoder_position").GetDouble(0));
		simState.SetRawRotorPosition(motorPosition);

		units::turns_per_second_t motorSpeed = units::turns_per_second_t(ntable->GetEntry("encoder_speed").GetDouble(0));
		simState.SetRotorVelocity(motorSpeed);
	}

	ntInst.Flush();
}