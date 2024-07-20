// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "OvertureLib/Subsystems/LedsManager/LedsManager.h"
#include <iostream>

LedsManager::LedsManager(int pwmPort, int ledLength,
		const std::map<LedStripName, LedStripRange> &ledStripMap) : ledStrip(
		pwmPort), ledStripMap(ledStripMap) {
	for (auto ledStrip : ledStripMap) {
		if (ledStrip.second.endLed <= ledStrip.second.startLed) {
			throw std::logic_error(
					"Led strip has an end led that is before the start led!!!");
		}

		if (ledStrip.second.startLed >= ledLength
				|| ledStrip.second.endLed >= ledLength) {
			throw std::logic_error(
					"Led strip has a start or end led that is greater than the total length!!!");
		}
	}

	ledBuffer.resize(ledLength);
	ledBufferSpan = { ledBuffer };

	ledStrip.SetLength(ledLength);
	ledStrip.SetData(ledBuffer);
	ledStrip.Start();
}
;

std::span<frc::AddressableLED::LEDData> LedsManager::getLedStrip(
		LedStripName name) {
	if (!ledStripMap.contains(name)) {
		return {};
	}

	const auto ledStrip = ledStripMap.at(name);
	return ledBufferSpan.subspan(ledStrip.startLed,
			ledStrip.endLed - ledStrip.startLed + 1);
}

// This method will be called once per scheduler run
void LedsManager::Periodic() {
	ledStrip.SetData(ledBuffer);
}
