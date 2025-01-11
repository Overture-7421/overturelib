// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "OvertureLib/Subsystems/Swerve/SwerveModule/SwerveModule.h"
#include <frc/RobotController.h>
#include <iostream>

SwerveModule::SwerveModule(SwerveModuleConfig config) : config(config), driveMotor(
		config.DriveMotorConfig, config.CanBus), turnMotor(
		config.TurnMotorConfig, config.CanBus), canCoder(config.EncoderConfig,
		config.CanBus), feedForward(config.FeedForward) {
	turnMotor.setContinuousWrap();
	turnMotor.setFusedCANCoder(config.EncoderConfig.CanCoderId);
	turnMotor.SetControl(
			turnVoltage.WithPosition(0_tr).WithEnableFOC(
					config.TurnMotorConfig.useFOC));

	driveMotor.SetPosition(0_tr);

	turnMotor.setPositionUpdateFrequency(200_Hz);
	canCoder.GetPosition().SetUpdateFrequency(200_Hz);
	driveMotor.setVelocityUpdateFrequency(200_Hz);

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
	state.Optimize(latestState.angle);
	state.CosineScale(latestState.angle);
	
	targetState = state;

	turnMotor.SetControl(
			turnVoltage.WithPosition(targetState.angle.Degrees()).WithEnableFOC(
					config.TurnMotorConfig.useFOC).WithSlot(0));
	driveMotor.SetControl(
			driveVoltage.WithOutput(feedForward.Calculate(targetState.speed)).WithEnableFOC(
					config.DriveMotorConfig.useFOC));
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
	driveMotor.SetControl(
			driveVoltage.WithOutput(volts).WithEnableFOC(
					config.DriveMotorConfig.useFOC));
}

units::volt_t SwerveModule::getVoltageDrive() {
	return driveMotor.GetMotorVoltage().GetValue();
}

/**
 * @brief Shuffleboard Periodic
 */
void SwerveModule::shuffleboardPeriodic() {
	frc::SmartDashboard::PutNumber(
			"SwerveChassis/Modules/" + config.ModuleName + "/TargetSpeed",
			targetState.speed.value());
	frc::SmartDashboard::PutNumber(
			"SwerveChassis/Modules/" + config.ModuleName + "/Speed",
			latestState.speed.value());
	frc::SmartDashboard::PutNumber(
			"SwerveChassis/Modules/" + config.ModuleName + "/TargetAngle",
			targetState.angle.Degrees().value());
	frc::SmartDashboard::PutNumber(
			"SwerveChassis/Modules/" + config.ModuleName + "/Angle",
			latestState.angle.Degrees().value());
	frc::SmartDashboard::PutNumber(
			"SwerveChassis/Modules/" + config.ModuleName + "/Distance",
			latestPosition.distance.value());
	frc::SmartDashboard::PutNumber(
			"SwerveChassis/Modules/" + config.ModuleName + "/RequestedVoltage",
			feedForward.Calculate(targetState.speed).value());
	frc::SmartDashboard::PutNumber(
			"SwerveChassis/Modules/" + config.ModuleName + "/AppliedVoltage",
			driveMotor.GetMotorVoltage().GetValueAsDouble());
}

void SwerveModule::Periodic() {
	units::degree_t angle = canCoder.GetAbsolutePosition().GetValue();
	latestState.speed = units::meters_per_second_t(
			driveMotor.GetVelocity().GetValueAsDouble()
					* config.WheelDiameter.value() * M_PI);
	latestState.angle = angle;

	latestPosition.distance = units::meter_t {
			driveMotor.GetPosition().GetValueAsDouble()
					* config.WheelDiameter.value() * M_PI };
	latestPosition.angle = angle;
}
