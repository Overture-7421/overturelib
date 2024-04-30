// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#pragma once

#include <frc2/command/sysid/SysIdRoutine.h>
#include <frc/DataLogManager.h>
#include <fmt/format.h>

#include "Subsystems/Swerve/SwerveChassis/SwerveChassis.h"

class SwerveCharacterization : public SwerveChassis {
public:
	SwerveCharacterization(units::meters_per_second_t maxModuleSpeed, units::meter_t driveBaseRadius);

	frc2::CommandPtr SysIdQuadstatic(frc2::sysid::Direction direction);
	frc2::CommandPtr SysIdDinamic(frc2::sysid::Direction direction);
	void sysIdVoltage(units::volt_t voltage);
private:
	frc2::sysid::SysIdRoutine m_sysIdRoutine{
		  frc2::sysid::Config{std::nullopt, std::nullopt, std::nullopt, std::nullopt},
		  frc2::sysid::Mechanism{
			  [this](units::volt_t driveVoltage) {
				sysIdVoltage(driveVoltage);
			  },
			  [this](frc::sysid::SysIdRoutineLog* log) {
				log->Motor("frontRight")
					.voltage(units::volt_t{frontRightModule->getVoltage()})
					.position(units::meter_t{frontRightModule->getDistance()})
					.velocity(units::meters_per_second_t{frontRightModule->getSpeed()});

				// log->Motor("frontLeft")
				// 	.voltage(units::volt_t{frontLeftModule->getVoltage()})
				// 	.position(units::meter_t{frontLeftModule->getDistance()})
				// 	.velocity(units::meters_per_second_t{frontLeftModule->getSpeed()});
				// log->Motor("backRight")
				// 	.voltage(units::volt_t{backRightModule->getVoltage()})
				// 	.position(units::meter_t{backRightModule->getDistance()})
				// 	.velocity(units::meters_per_second_t{backRightModule->getSpeed()});
				// log->Motor("backLeft")
				// 	.voltage(units::volt_t{backLeftModule->getVoltage()})
				// 	.position(units::meter_t{backLeftModule->getDistance()})
				// 	.velocity(units::meters_per_second_t{backLeftModule->getSpeed()});
			  },
			  this} };
};
