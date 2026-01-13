// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "OvertureLib/Sensors/OverCANCoder/OverCANCoder.h"

#ifndef __FRC_ROBORIO__
#include "OvertureLib/Simulation/SimCANCoderManager/SimCANCoderManager.h"
#endif

/**
 * @brief Constructor for OverCANCoder
 *
 * @param _id    CAN ID of the CANCoder
 *
 * @param offset Offset of the CANCoder
 *
 * @param bus    CAN Bus of the CANCoder
 */
OverCANCoder::OverCANCoder(CanCoderConfig config, ctre::phoenix6::CANBus bus) : ctre::phoenix6::hardware::CANcoder(
		config.CanCoderId, bus) {

	canCoderConfiguration.MagnetSensor.WithAbsoluteSensorDiscontinuityPoint(
			config.absoluteDiscontinuityPoint);
	canCoderConfiguration.MagnetSensor.WithMagnetOffset(config.Offset);
	canCoderConfiguration.MagnetSensor.WithSensorDirection(
			config.SensorDirection);

	GetConfigurator().Apply(canCoderConfiguration);

#ifndef __FRC_ROBORIO__
	SimCANCoderManager &simCANCoderManager = SimCANCoderManager::GetInstance();
	simCANCoderManager.AddSimCANCoderCandidate(this);
#endif

	isConnectedAlert.SetText(
			"CANCoder " + std::to_string(config.CanCoderId)
					+ " is not connected");
}

const ctre::phoenix6::configs::CANcoderConfiguration& OverCANCoder::getConfiguration() {
	return canCoderConfiguration;
}
