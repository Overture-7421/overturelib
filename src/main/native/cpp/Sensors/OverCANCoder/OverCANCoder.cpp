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
OverCANCoder::OverCANCoder(int _id, units::turn_t offset, std::string _bus) : CANcoder(_id, _bus) {
	canCoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue::Signed_PlusMinusHalf;
	canCoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue::CounterClockwise_Positive;
	canCoderConfiguration.MagnetSensor.MagnetOffset = offset.value();
	GetConfigurator().Apply(canCoderConfiguration);

#ifndef __FRC_ROBORIO__
	SimCANCoderManager* simCANCoderManager = SimCANCoderManager::GetInstance();
	simCANCoderManager->AddSimCANCoderCandidate(this);
#endif
}

/**
* @brief Gets the CANCoder's absolute position
*
* @return CANCoder's absolute position
*/
double OverCANCoder::getSensorAbsolutePosition() {
	return GetAbsolutePosition().GetValue().value();
}

const CANcoderConfiguration& OverCANCoder::getConfiguration() {
	return canCoderConfiguration;
}
