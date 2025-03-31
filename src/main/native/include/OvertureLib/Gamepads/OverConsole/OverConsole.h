// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/geometry/Rotation2d.h>
#include <frc/Alert.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/Trigger.h>
#include <frc2/command/button/CommandGenericHID.h>
#include "OvertureLib/Math/Utils.h"
#include "OvertureLib/Utils/Logging/Logging.h"

#pragma once

class OverConsole: public frc2::CommandGenericHID {
public:
	OverConsole(int port);
	void updateTelemetry();

private:
	frc::Alert isConnectedAlert { "Controllers", "Console is not connected",
			frc::Alert::AlertType::kWarning };
};
