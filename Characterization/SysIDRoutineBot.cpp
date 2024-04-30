// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SysIDRoutineBot.h"

#include <frc2/command/Commands.h>

SysIDRoutineBot::SysIDRoutineBot() {}

void SysIDRoutineBot::ConfigureSysIdBindings(SwerveCharacterization* m_drive, frc2::CommandXboxController* m_driverController) {

	m_driverController->A().WhileTrue(
		m_drive->SysIdQuadstatic(frc2::sysid::Direction::kForward));
	m_driverController->B().WhileTrue(
		m_drive->SysIdQuadstatic(frc2::sysid::Direction::kReverse));
	m_driverController->X().WhileTrue(
		m_drive->SysIdDinamic(frc2::sysid::Direction::kForward));
	m_driverController->Y().WhileTrue(
		m_drive->SysIdDinamic(frc2::sysid::Direction::kReverse));
}