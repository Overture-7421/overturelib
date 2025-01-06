// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "OvertureLib/Simulation/SimCANCoderManager/SimCANCoderManager.h"
#include <iostream>
#include <frc/RobotController.h>
#include <frc/MathUtil.h>

SimCANCoderManager::SimCANCoderManager() {

}

void SimCANCoderManager::AddSimCANCoderCandidate(OverCANCoder *canCoder) {
		std::cout << "Adding sim cancoder candidate with id: " << canCoder->GetDeviceID()
			<< "..." << std::endl;

	canCodersToRegister.emplace_back(canCoder);
}

void SimCANCoderManager::RegisterSimCANCoder(OverCANCoder *canCoder) {
	if (canCoder == NULL || canCoder == 0) {
		throw std::invalid_argument("SimCANCoderManager given null pointer!");
	}

	if (!this->CANIDToCANCoderNameMap.contains(canCoder->GetDeviceID())) {
		std::cout
				<< "SimCANCoderManager Warning: Tried to register a CANCoder for simulation("
				<< canCoder->GetDeviceID() << ") that is not in the given map"
				<< std::endl;
		return;
	}

	std::string canCoderName =
			this->CANIDToCANCoderNameMap[canCoder->GetDeviceID()];

	if (this->registeredCANCoders.contains(canCoderName)) {
		std::cout
				<< "SimCANCoderManager Warning: Tried to register a motor for simulation that was already registered"
				<< std::endl;
		return;
	}

	std::shared_ptr < nt::NetworkTable > ntable = ntInst.GetTable(canCoderName);

	CANCoderNTPair newCANCoderPair;
	newCANCoderPair.canCoder = canCoder;
	newCANCoderPair.ntable = ntable;

	this->registeredCANCoders[canCoderName] = std::move(newCANCoderPair);
}

void SimCANCoderManager::Init(
		const std::map<unsigned int, NTCANCoderName> CANIDToCANCoderNameMap) {
	std::cout << "Initializing SimCANCoderManager..." << std::endl;
	this->CANIDToCANCoderNameMap = CANIDToCANCoderNameMap;
	std::cout << "Got " << this->CANIDToCANCoderNameMap.size()
			<< " mapped CANCoders..." << std::endl;
	
	std::cout << "Got " << this->canCodersToRegister.size()
			<< " CANCoders to register..." << std::endl;

	std::for_each(canCodersToRegister.begin(), canCodersToRegister.end(),
			std::bind(&SimCANCoderManager::RegisterSimCANCoder, this,
					std::placeholders::_1));
	initialized = true;
}

void SimCANCoderManager::Update() {
	if (!initialized) {
		return;
	}

	for (auto canCoderIterator = this->registeredCANCoders.begin();
			canCoderIterator != this->registeredCANCoders.end();
			canCoderIterator++) {
		CANCoderNTPair pair = canCoderIterator->second;

		std::shared_ptr < nt::NetworkTable > ntable = pair.ntable;
		OverCANCoder *canCoder = pair.canCoder;

		ctre::phoenix6::sim::CANcoderSimState &simState =
				canCoder->GetSimState();
		simState.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

		units::turn_t position = units::turn_t(
				ntable->GetEntry("cancoder_position").GetDouble(0));
		simState.SetRawPosition(position);
		units::turns_per_second_t speed = units::turns_per_second_t(
				ntable->GetEntry("cancoder_speed").GetDouble(0));
		simState.SetVelocity(speed);
	}

	ntInst.Flush();
}

SimCANCoderManager& SimCANCoderManager::GetInstance() {
	static SimCANCoderManager instance;
	return instance;
}
