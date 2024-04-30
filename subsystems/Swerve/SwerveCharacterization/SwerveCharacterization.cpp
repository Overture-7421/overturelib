// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SwerveCharacterization.h"

SwerveCharacterization::SwerveCharacterization(units::meters_per_second_t maxModuleSpeed, units::meter_t driveBaseRadius) : SwerveChassis(maxModuleSpeed, driveBaseRadius) {

};

/**
 * @brief Runs the SysId Quasisstatic command
*/
frc2::CommandPtr SwerveCharacterization::SysIdQuadstatic(frc2::sysid::Direction direction) {
	return frc2::cmd::Sequence(
		frc2::InstantCommand([this]() { sysIdVoltage(0_V); }).ToPtr(),
		frc2::cmd::Wait(0.5_s),
		m_sysIdRoutine.Quasistatic(direction)
	);
}

/**
 * @brief Runs the SysId Dynamic command
*/
frc2::CommandPtr SwerveCharacterization::SysIdDinamic(frc2::sysid::Direction direction) {
	return frc2::cmd::Sequence(
		frc2::InstantCommand([this]() { sysIdVoltage(0_V); }).ToPtr(),
		frc2::cmd::Wait(0.5_s),
		m_sysIdRoutine.Dynamic(direction)
	);
}

/**
 * @brief Sets the voltage for the SysId command
*/
void SwerveCharacterization::sysIdVoltage(units::volt_t voltage) {
	frontLeftModule->setRawVoltageSpeed(voltage);
	frontRightModule->setRawVoltageSpeed(voltage);
	backLeftModule->setRawVoltageSpeed(voltage);
	backRightModule->setRawVoltageSpeed(voltage);

}
