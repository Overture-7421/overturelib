// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SwerveModule.h"

/**
 * @brief Construye un módulo de Swerve
*/
SwerveModule::SwerveModule(int rotatorID, int wheelID, int canCoderID, double offset, std::string moduleName, std::string canBus): m_name(moduleName) {
    m_driveMotor = new OverTalonFX(wheelID, ControllerNeutralMode::Brake, true, 1, canBus);
    m_turningMotor = new OverTalonFX(rotatorID, ControllerNeutralMode::Coast, true, 1, canBus);
    m_canCoder = new OverCANCoder(canCoderID, offset, canBus);

    m_turningMotor->setContinuousWrap();
    m_turningMotor->setFusedCANCoder(canCoderID);
    m_turningMotor->setClosedLoopVoltageRamp(0.1);
    m_turningMotor->setSupplyCurrentLimit(true, 20, 30, 0.5);
    m_turningMotor->setPositionVoltage(0, false);

    m_driveMotor->zeroPosition();
    m_driveMotor->setClosedLoopTorqueRamp(0.1);
    m_driveMotor->setTorqueCurrentLimit(40, -40, 0.1);
    // m_driveMotor->setClosedLoopVoltageRamp(0.1);
    // m_driveMotor->setSupplyCurrentLimit(true, 20, 30, 0.5);
}

/**
 * @brief Establece los valores de PID para el motor rotatorio
 * @param kP Constante proporcional
 * @param kI Constante integral
 * @param kD Constante derivativa
 */
void SwerveModule::setRotatorPIDValues(double kP, double kI, double kD) {
    m_turningMotor->setPIDValues(kP, kI, kD, 0, 0);
}

/**
 * @brief Establece los valores de PID para el motor de velocidad
 * @param kP Constante proporcional
 * @param kI Constante integral
 * @param kD Constante derivativa
 * @param kS Constante de feedforward estático
 * @param kV Constante de feedforward dinámico
 */
void SwerveModule::setDrivePIDValues(double kP, double kI, double kD) {
    m_driveMotor->setPIDValues(kP, kI, kD, 0, 0);
}

/**
 * @brief Establece los valores de feedforward para el motor de velocidad
 * @param ks Constante de feedforward estático
 * @param kv Constante de feedforward dinámico
 * @param ka Constante de feedforward de aceleración
 */
void SwerveModule::setFFConstants(units::volt_t ks, units::volt_t kv, units::volt_t ka) {
    m_feedForward = frc::SimpleMotorFeedforward<units::meters>(ks, kv / 1_mps, ka / 1_mps_sq);
}

/**
 * @brief Establece la relación de engranajes del motor rotatorio
 * @param _gearRatio Relación de engranajes
 */
void SwerveModule::setGearRatio(double _turn, double _wheel) {
    m_turningMotor->setRotorToSensorRatio(_turn);
    m_driveMotor->setSensorToMechanism(_wheel);
}

/**
 * @brief Establece el diámetro de la rueda
 * @param _wheelDiameter Diámetro de la rueda en metros
 */
void SwerveModule::setWheelDiameter(double _wheelDiameter) {
    this->m_wheelDiameter = _wheelDiameter;
}

/**
 * @brief Obtiene la velocidad de módulo
 * @return Velocidad del módulo
 */
double SwerveModule::getSpeed() {
    return m_driveMotor->getVelocity(m_wheelDiameter);
}

/**
 * @brief Pendiente a escribir
 */
double SwerveModule::setSpeed(double speed) {
    // return ((speed / (m_wheelDiameter * M_PI)));
    return 2.4 * speed;
}

/**
 * @brief Obtiene la distancia recorrida por el módulo
 * @return Distancia recorrida por el módulo
 */
double SwerveModule::getDistance() {
    return m_driveMotor->getDistance(m_wheelDiameter);
}

/**
 * @brief Obtiene el ángulo del módulo
 * @return Ángulo del módulo
 */
double SwerveModule::getAngle() {
    return m_canCoder->getAbsolutePosition() * 360;
}

/**
 * @brief Obtiene el estado del módulo
 * @return Estado del módulo
 */
frc::SwerveModuleState SwerveModule::getState() {
    frc::SwerveModuleState state;

    state.speed = units::meters_per_second_t(getSpeed());
    state.angle = units::degree_t(getAngle());

    return state;
}

/**
 * @brief Establece el estado del módulo
 * @param state Estado del módulo
 */
void SwerveModule::setState(frc::SwerveModuleState state) {
    m_state = frc::SwerveModuleState::Optimize(state, m_state.angle);
}

/**
 * @brief Obtiene la posición del módulo
 * @return Posición del módulo
 */
frc::SwerveModulePosition SwerveModule::getPosition() {
    return { units::meter_t{getDistance()}, units::degree_t{getAngle()} };
}

/**
 * @brief Le da un valor de velocidad al motor
 * @param wheelVoltage Voltaje del motor
 */
void SwerveModule::setWheelVoltage(double wheelVoltage) {
    this->wheelVoltage = units::volt_t{ wheelVoltage };
}

/**
 * @brief Le da valores a los dos motores
 */
void SwerveModule::setVoltages() {
    m_turningMotor->setPositionVoltage(m_state.angle.Degrees().value() / 360.0, true);

    if (useRawVoltageSpeed) {
        m_driveMotor->setVoltage(wheelVoltage, true);
    } else {
        // m_driveMotor->setVoltage(m_feedForward.Calculate(m_state.speed), true);
        m_driveMotor->setVoltage(units::volt_t{ setSpeed(m_state.speed.value()) }, true);
        // m_driveMotor->setVelocityTorqueCurrentFOC(setSpeed(m_state.speed.vale()));
    }
}
// This method will be called once per scheduler run
void SwerveModule::Periodic() {
    frc::SmartDashboard::PutNumber(m_name + "/Speed", getSpeed());
    frc::SmartDashboard::PutNumber(m_name + "/Target", m_state.angle.Degrees().value());
    frc::SmartDashboard::PutNumber(m_name + "/Angle", getAngle());
}
