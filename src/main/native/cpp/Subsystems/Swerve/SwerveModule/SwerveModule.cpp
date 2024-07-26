// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "OvertureLib/Subsystems/Swerve/SwerveModule/SwerveModule.h"

/**
 * @brief Swerve Module Constructor
 *
 * @param rotatorID  - ID of the motor that rotates the wheel
 * @param wheelID    - ID of the motor that drives the wheel
 * @param canCoderID - ID of the canCoder that is attached to the rotator
 * @param offset     - Offset of the canCoder
 * @param moduleName - Name of the module
 * @param canBus     - Can Bus of the module
 */
SwerveModule::SwerveModule(ModuleConfig config) : m_name(config.moduleName) {
	m_driveMotor =
			std::make_unique < OverTalonFX
					> (config.rotatorId, ControllerNeutralMode::Brake, false, config.canBus);
	m_turningMotor =
			std::make_unique < OverTalonFX
					> (config.wheelId, ControllerNeutralMode::Coast, true, config.canBus);
	m_canCoder = std::make_unique < OverCANCoder
			> (config.canCoderId, config.offset, config.canBus);
	m_turningMotor->setContinuousWrap();
	m_turningMotor->setFusedCANCoder(config.canCoderId);
	m_turningMotor->setClosedLoopVoltageRamp(0.1);
	m_turningMotor->setSupplyCurrentLimit(true, 20, 30, 0.5);
	m_turningMotor->setPositionVoltage(0, false);

	m_driveMotor->zeroPosition();
	m_driveMotor->setClosedLoopVoltageRamp(0.1);
	m_driveMotor->setSupplyCurrentLimit(true, 40, 60, 0.1);

	m_turningMotor->setPositionUpdateFrequency(200_Hz);
	m_canCoder->GetPosition().SetUpdateFrequency(200_Hz);
	m_driveMotor->setVelocityUpdateFrequency(200_Hz);

	// Set PID Values
	m_turningMotor->setPIDValues(config.kP, config.kI, config.kD, 0, 0);

	// Set FeedForward Values
	m_feedForward = std::make_shared < frc::SimpleMotorFeedforward
			< units::meters
					>> (config.ks, config.kv / 1_mps, config.ka / 1_mps_sq);

	// Set Gear Ratios
	m_turningMotor->setRotorToSensorRatio(config.turnGearRatio);
	m_driveMotor->setSensorToMechanism(config.wheelGearRatio);

	// Set Wheel Diameter
	m_wheelDiameter = config.wheelDiameter;
}

/**
 * @brief Gets the speed of the wheel
 *
 * @return - Speed of the wheel
 */
double SwerveModule::getSpeed() {
	return m_driveMotor->getVelocity(m_wheelDiameter);
}

/**
 * @brief Set Module Speed
 *
 * @param speed - Speed of the wheel
 */
double SwerveModule::setSpeed(double speed) {
	return ((speed / (m_wheelDiameter * M_PI)));
}

/**
 * @brief Gets the distance traveled by the module
 *
 * @return - Distance traveled by the module
 */
double SwerveModule::getDistance() {
	return m_driveMotor->getDistance(m_wheelDiameter);
}

/**
 * @brief Obtiene el ángulo del módulo
 * @return Ángulo del módulo
 */
double SwerveModule::getAngle() {
	return m_canCoder->getSensorAbsolutePosition() * 360;
}

/**
 * @brief Gets the voltage of the module
 *
 * @return - Voltage of the module
 */
double SwerveModule::getVoltage() {
	return m_driveMotor->GetMotorVoltage().GetValueAsDouble();
}

/**
 * @brief Gets the state of the module
 *
 * @return - State of the module
 */
frc::SwerveModuleState SwerveModule::getState() {
	frc::SwerveModuleState state;

	state.speed = units::meters_per_second_t(getSpeed());
	state.angle = units::degree_t(getAngle());

	return state;
}

/**
 * @brief Sets the state of the module
 *
 * @param state - State of the module
 */
void SwerveModule::setState(frc::SwerveModuleState state) {
	m_state = frc::SwerveModuleState::Optimize(state, m_state.angle);
}

/**
 * @brief Gets the module position
 *
 * @return - Module position
 */
frc::SwerveModulePosition SwerveModule::getPosition() {
	return {units::meter_t {getDistance()}, units::degree_t {getAngle()}};
}

/**
 * @brief Sets the raw voltage speed
 *
 * @param volts - Voltage
 */
void SwerveModule::setRawVoltageSpeed(units::volt_t volts) {
	m_driveMotor->setVoltage(volts, false);

	m_turningMotor->setPositionVoltage(0, false);
}

/**
 * @brief Sets the voltage of the module
 */
void SwerveModule::setVoltages() {
	m_turningMotor->setPositionVoltage(m_state.angle.Degrees().value() / 360.0,
			false);

	m_driveMotor->setVoltage(m_feedForward->Calculate(m_state.speed), false);
	//m_driveMotor->setVoltage(units::volt_t{ setSpeed(m_state.speed.value()) }, false);
}

/**
 * @brief Shuffleboard Periodic
 */
void SwerveModule::shuffleboardPeriodic() {
	frc::SmartDashboard::PutNumber(m_name + "/Speed", getSpeed());
	frc::SmartDashboard::PutNumber(m_name + "/Target",
			m_state.angle.Degrees().value());
	frc::SmartDashboard::PutNumber(m_name + "/Angle", getAngle());
	frc::SmartDashboard::PutNumber(m_name + "/Distance", getDistance());
}

void SwerveModule::Periodic() {
}
