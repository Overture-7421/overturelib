// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>

RobotContainer::RobotContainer() {
  frc::SmartDashboard::PutData("Auto Chooser", &pathChooser);
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  return pathChooser.GetSelected();
}
