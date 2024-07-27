// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "OvertureLib/Simulation/SimDutyCycleEncoderManager/SimDutyCycleEncoderManager.h"
#include <iostream>

SimDutyCycleEncoderManager::SimDutyCycleEncoderManager() {

}

void SimDutyCycleEncoderManager::AddSimDutyCycleEncoderCandidate(
		OverDutyCycleEncoder *encoder) {
	dutyCycleEncodersToRegister.emplace_back(encoder);
}

void SimDutyCycleEncoderManager::RegisterSimDutyCycleEncoder(
		OverDutyCycleEncoder *encoder) {
	if (encoder == NULL || encoder == 0) {
		throw std::invalid_argument(
				"SimDutyCycleEncoderManager given null pointer!");
	}

	if (!this->CHNToDutyCycleEncoderNameMap.contains(
			encoder->GetSourceChannel())) {
		std::cout
				<< "SimDutyCycleEncoderManager Warning: Tried to register a DutyCycleEncoder for simulation("
				<< encoder->GetSourceChannel()
				<< ") that is not in the given map" << std::endl;
		return;
	}

	std::string encoderName =
			this->CHNToDutyCycleEncoderNameMap[encoder->GetSourceChannel()];

	if (this->registeredDutyCycleEncoders.contains(encoderName)) {
		std::cout
				<< "SimDutyCycleEncoderManager Warning: Tried to register a DutyCycleEncoder for simulation that was already registered"
				<< std::endl;
		return;
	}

	std::shared_ptr < nt::NetworkTable > ntable = ntInst.GetTable(encoderName);

	DutyCycleEncoderNTPair newDutyCycleEncoderPair;
	newDutyCycleEncoderPair.dutyCycleEncoder = std::make_shared
			< frc::sim::DutyCycleEncoderSim
			> (frc::sim::DutyCycleEncoderSim(*encoder));
	newDutyCycleEncoderPair.ntable = ntable;

	newDutyCycleEncoderPair.dutyCycleEncoder->SetConnected(true);
	this->registeredDutyCycleEncoders[encoderName] = std::move(
			newDutyCycleEncoderPair);
}

void SimDutyCycleEncoderManager::Init(
		const std::map<unsigned int, NTDutyCycleEncoderName> CHNToDutyCycleEncoderNameMap) {
	this->CHNToDutyCycleEncoderNameMap = CHNToDutyCycleEncoderNameMap;

	std::for_each(dutyCycleEncodersToRegister.begin(),
			dutyCycleEncodersToRegister.end(),
			std::bind(&SimDutyCycleEncoderManager::RegisterSimDutyCycleEncoder,
					this, std::placeholders::_1));
	initialized = true;
}

void SimDutyCycleEncoderManager::Update() {
	if (!initialized) {
		return;
	}

	for (auto encoderIterator = this->registeredDutyCycleEncoders.begin();
			encoderIterator != this->registeredDutyCycleEncoders.end();
			encoderIterator++) {
		DutyCycleEncoderNTPair pair = encoderIterator->second;

		std::shared_ptr < nt::NetworkTable > ntable = pair.ntable;
		std::shared_ptr < frc::sim::DutyCycleEncoderSim > encoder =
				pair.dutyCycleEncoder;

		// units::turn_t position = units::turn_t();
		encoder->SetAbsolutePosition(
				ntable->GetEntry("cancoder_position").GetDouble(0));
		encoder->SetConnected(ntable->GetEntry("cancoder_position").Exists());
	}

	ntInst.Flush();
}

SimDutyCycleEncoderManager& SimDutyCycleEncoderManager::GetInstance() {
	static SimDutyCycleEncoderManager instance;
	return instance;
}
