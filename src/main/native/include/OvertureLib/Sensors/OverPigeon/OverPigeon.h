// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/phoenix6/Pigeon2.hpp>
#include <ctre/phoenix6/CANBus.hpp>
#include <frc/Alert.h>

class OverPigeon: public ctre::phoenix6::hardware::Pigeon2 {
public:
	OverPigeon(int deviceId, ctre::phoenix6::CANBus canbus =
			ctre::phoenix6::CANBus { "" });

private:
	frc::Alert isConnectedAlert { "Devices", "Pigeon is not connected",
			frc::Alert::AlertType::kWarning };
};
