// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "OvertureLib/Gamepads/OverXboxController/OverXboxController.h"
#include <frc2/command/Commands.h>

frc2::CommandPtr WaitForButton(OverXboxController *gamepad, int buttonNumber);

frc2::CommandPtr WaitForButton(frc2::CommandGenericHID *gamepad, int buttonNumber);

