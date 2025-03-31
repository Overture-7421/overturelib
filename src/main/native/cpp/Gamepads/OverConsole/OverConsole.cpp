// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "OvertureLib/Gamepads/OverConsole/OverConsole.h"

OverConsole::OverConsole(int port) : frc2::CommandGenericHID(port) {

}

void OverConsole::updateTelemetry() {
	if (IsConnected()) {
		isConnectedAlert.Set(true);
	} else {
		isConnectedAlert.Set(false);
	}

	Logging::WriteBoolean(
			"Controllers/ConsolePad-" + std::to_string(GetHID().GetPort())
					+ "/IsConnected", IsConnected());
}
