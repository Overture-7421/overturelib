// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/CANBus.hpp>

#include <OvertureLib/Sensors/OverCANCoder/Config.h>
#include <frc/Alert.h>

class OverCANCoder: public ctre::phoenix6::hardware::CANcoder {
public:
	OverCANCoder(CanCoderConfig config, ctre::phoenix6::CANBus bus);
	const ctre::phoenix6::configs::CANcoderConfiguration& getConfiguration();

private:
	ctre::phoenix6::configs::CANcoderConfiguration canCoderConfiguration;
	frc::Alert isConnectedAlert { "Devices", "CANCoder is not connected",
			frc::Alert::AlertType::kWarning };
};
