// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "OvertureLib/Subsystems/Swerve/SwerveModule/SwerveModule.h"
#include <iostream>

SwerveModule::SwerveModule(ModuleConfig config) : config(config), driveMotor(
		config.DrivedId, config.DriveNeutralMode, config.DriveInverted,
		config.CanBus), turnMotor(config.TurnId, config.TurnNeutralMode,
		config.TurnInverted, config.CanBus), canCoder(config.CanCoderId,
		config.Offset, config.CanBus), feedForward(config.FeedForward) {
	turnMotor.setContinuousWrap();
	turnMotor.setFusedCANCoder(config.CanCoderId);
	turnMotor.setClosedLoopVoltageRamp(0.1);
	turnMotor.setSupplyCurrentLimit(true, 20, 30, 0.5);
	turnMotor.setPositionVoltage(0, false);

	driveMotor.zeroPosition();
	driveMotor.setClosedLoopVoltageRamp(0.1);
	driveMotor.setSupplyCurrentLimit(true, 40, 60, 0.1);

	turnMotor.setPositionUpdateFrequency(200_Hz);
	canCoder.GetPosition().SetUpdateFrequency(200_Hz);
	driveMotor.setVelocityUpdateFrequency(200_Hz);

	// Set PID Values
	turnMotor.setPIDValues(config.kP, config.kI, config.kD, 0, 0);

	// Set Gear Ratios
	turnMotor.setRotorToSensorRatio(config.TurnGearRatio);
	driveMotor.setSensorToMechanism(config.DriveGearRatio);
}

/**
 * @brief Gets the state of the module
 *
 * @return - State of the module
 */
const frc::SwerveModuleState& SwerveModule::getState() {
	return latestState;
}

/**
 * @brief Sets the state of the module
 *
 * @param state - State of the module
 */
void SwerveModule::setState(frc::SwerveModuleState state) {
	targetState = frc::SwerveModuleState::Optimize(state, getState().angle);
	turnMotor.setPositionVoltage(targetState.angle.Degrees().value() / 360.0,
			false);
	driveMotor.setVoltage(feedForward.Calculate(targetState.speed), false);
}

/**
 * @brief Gets the module position
 *
 * @return - Module position
 */
const frc::SwerveModulePosition& SwerveModule::getPosition() {
	return latestPosition;
}

/**
 * @brief Sets the raw voltage speed
 *
 * @param volts - Voltage
 */
void SwerveModule::setVoltageDrive(units::volt_t volts) {
	driveMotor.setVoltage(volts, false);
}

units::volt_t SwerveModule::getVoltageDrive() {
	return driveMotor.GetMotorVoltage().GetValue();
}

/**
 * @brief Shuffleboard Periodic
 */
void SwerveModule::shuffleboardPeriodic() {
	frc::SmartDashboard::PutNumber(config.ModuleName + "/Speed",
			latestState.speed.value());
	frc::SmartDashboard::PutNumber(config.ModuleName + "/Target",
			targetState.angle.Degrees().value());
	frc::SmartDashboard::PutNumber(config.ModuleName + "/Angle",
			latestState.angle.Degrees().value());
	frc::SmartDashboard::PutNumber(config.ModuleName + "/Distance",
			latestPosition.distance.value());
}

void SwerveModule::Periodic() {
	units::degree_t angle = units::degree_t(
			canCoder.getSensorAbsolutePosition() * 360.0);
	latestState.speed = units::meters_per_second_t(
			driveMotor.getVelocity(config.WheelDiameter.value()));
	latestState.angle = angle;

	latestPosition.distance = units::meter_t { driveMotor.getDistance(
			config.WheelDiameter.value()) };
	latestPosition.angle = angle;
}
