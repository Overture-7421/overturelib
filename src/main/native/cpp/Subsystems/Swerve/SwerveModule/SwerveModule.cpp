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
SwerveModule::SwerveModule(int rotatorID, int wheelID, int canCoderID,
		units::turn_t offset, std::string moduleName, std::string canBus) : m_name(
		moduleName) {
	m_driveMotor = std::make_unique < OverTalonFX
			> (wheelID, ControllerNeutralMode::Brake, false, canBus);
	m_turningMotor = std::make_unique < OverTalonFX
			> (rotatorID, ControllerNeutralMode::Coast, true, canBus);
	m_canCoder = std::make_unique < OverCANCoder > (canCoderID, offset, canBus);
	m_turningMotor->setContinuousWrap();
	m_turningMotor->setFusedCANCoder(canCoderID);
	m_turningMotor->setClosedLoopVoltageRamp(0.1);
	m_turningMotor->setSupplyCurrentLimit(true, 20, 30, 0.5);
	m_turningMotor->setPositionVoltage(0, false);

	m_driveMotor->zeroPosition();
	// m_driveMotor->setClosedLoopTorqueRamp(0.1);
	// m_driveMotor->setTorqueCurrentLimit(40, -40, 0.1);
	m_driveMotor->setClosedLoopVoltageRamp(0.1);
	m_driveMotor->setSupplyCurrentLimit(true, 40, 60, 0.1);

	m_turningMotor->setPositionUpdateFrequency(200_Hz);
	m_canCoder->GetPosition().SetUpdateFrequency(200_Hz);
	m_driveMotor->setVelocityUpdateFrequency(200_Hz);

	setFFConstants(0_V, 0_V, 0_V);
}

/**
 * @brief Sets Rotator Motor PID Values
 *
 * @param kP - Proportional Value
 * @param kI - Integral Value
 * @param kD - Derivative Value
 */
void SwerveModule::setRotatorPIDValues(double kP, double kI, double kD) {
	m_turningMotor->setPIDValues(kP, kI, kD, 0, 0);
}

/**
 * @brief Sets Drive Motor PID Values
 *
 * @param kP - Proportional Value
 * @param kI - Integral Value
 * @param kD - Derivative Value
 */
void SwerveModule::setDrivePIDValues(double kP, double kI, double kD) {
	m_driveMotor->setPIDValues(kP, kI, kD, 0, 0);
}

/**
 * @brief Sets FeedForward Values
 *
 * @param kS - Static Value
 * @param kV - Velocity Value
 * @param kA - Acceleration Value
 */
void SwerveModule::setFFConstants(units::volt_t ks, units::volt_t kv,
		units::volt_t ka) {
	m_feedForward = std::make_shared < frc::SimpleMotorFeedforward
			< units::meters >> (ks, kv / 1_mps, ka / 1_mps_sq);
}

/**
 * @brief Sets the gear ratio of both motors
 *
 * @param turn
 * @param wheel
 */
void SwerveModule::setGearRatio(double turn, double wheel) {
	m_turningMotor->setRotorToSensorRatio(turn);
	m_driveMotor->setSensorToMechanism(wheel);
}

/**
 * @brief the wheel diameter
 *
 * @param wheelDiameter - Diameter of the wheel
 */
void SwerveModule::setWheelDiameter(double wheelDiameter) {
	this->m_wheelDiameter = wheelDiameter;
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

void SwerveModule::Periodic() {
	// frc::SmartDashboard::PutNumber(m_name + "/Speed", getSpeed());
	// frc::SmartDashboard::PutNumber(m_name + "/Target", m_state.angle.Degrees().value());
	// frc::SmartDashboard::PutNumber(m_name + "/Angle", getAngle());
	// frc::SmartDashboard::PutNumber(m_name + "/Distance", getDistance());

}
