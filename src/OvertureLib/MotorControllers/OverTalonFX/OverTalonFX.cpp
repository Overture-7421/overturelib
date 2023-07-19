// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "OverTalonFX.h"

OverTalonFX::OverTalonFX(int _id, ControllerNeutralMode _neutralMode, bool _inverted, double _gearRatio, std::string _bus):
    OverController(new TalonFX(_id, _bus), _neutralMode, _inverted, _gearRatio) {
    init();
}

/**
 * @brief configurar el TalonFX inicialmente
 */
void OverTalonFX::init() {
    // Configuracion en modo neutral
    setNeutralMode(neutralMode);

    talonFXConfiguration.Voltage.PeakForwardVoltage = 12;
    talonFXConfiguration.Voltage.PeakReverseVoltage = -12;

    // Configuracion modo inverso
    if (inverted) {
        talonFXConfiguration.MotorOutput.Inverted = 1;
    } else {
        talonFXConfiguration.MotorOutput.Inverted = 0;
    }

    // Aplicar la configuracion
    motorController->GetConfigurator().Apply(talonFXConfiguration);

}

/**
 * @brief Resetear el TalonFX
 */
void OverTalonFX::reset() {}

/**
 * @brief Cambiar el modo neutral del TalonFX
 * @param _neutralMode Modo neutral del TalonFX
 */
void OverTalonFX::setNeutralMode(ControllerNeutralMode _neutralMode) {
    // Cambiar el modo neutral
    if (_neutralMode == ControllerNeutralMode::Coast) {
        talonFXConfiguration.MotorOutput.NeutralMode = 0;
    } else {
        talonFXConfiguration.MotorOutput.NeutralMode = 1;
    }
}

/**
 * @brief Se establece la relacion de engranajes con el mecanismo del TalonFX
 * @param _gearRatio Relacion de engranajes del TalonFX
 * @note Se debe utilizar cuando se utiliza el sensor integrado del TalonFX
 */
void OverTalonFX::setSensorToMechanism(double _gearRatio) {
    talonFXConfiguration.Feedback.SensorToMechanismRatio = _gearRatio;
    motorController->GetConfigurator().Apply(talonFXConfiguration);
    gearRatio = _gearRatio;
}

/**
 * @brief Se establece la relacion de engranajes con el mecanismo del TalonFX
 * @param _gearRatio Relacion de engranajes del TalonFX
 * @note Se debe utilizar cuando se utiliza el sensor integrado del TalonFX
 */
void OverTalonFX::setRotorToSensorRatio(double _gearRatio) {
    talonFXConfiguration.Feedback.RotorToSensorRatio = _gearRatio;
    motorController->GetConfigurator().Apply(talonFXConfiguration);
    gearRatio = _gearRatio;
}

/**
 * @brief Se conecta con un cancoder remoto
 * @param _id id del cancoder remoto
 */
void OverTalonFX::setRemoteCANCoder(int _id) {
    talonFXConfiguration.Feedback.FeedbackRemoteSensorID = _id;
    talonFXConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue::RemoteCANcoder;
    motorController->GetConfigurator().Apply(talonFXConfiguration);
}

/**
 * @brief Se conecta con un cancoder remoto
 * @param _id id del cancoder remoto
 */
void OverTalonFX::setFusedCANCoder(int _id) {
    talonFXConfiguration.Feedback.FeedbackRemoteSensorID = _id;
    talonFXConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue::FusedCANcoder;
    motorController->GetConfigurator().Apply(talonFXConfiguration);
}

/**
 * @brief Se establece la rampa de voltaje del TalonFX
 * @param _ramp Rampa de voltaje del TalonFX
 */
void OverTalonFX::setClosedLoopVoltageRamp(double _ramp) {
    talonFXConfiguration.ClosedLoopRamps.VoltageClosedLoopRampPeriod = _ramp;
    motorController->GetConfigurator().Apply(talonFXConfiguration);
}

/**
 * @brief Se establece la rampa de torque del TalonFX
 * @param _ramp Rampa de torque del TalonFX
 */
void OverTalonFX::setClosedLoopTorqueRamp(double _ramp) {
    talonFXConfiguration.ClosedLoopRamps.TorqueClosedLoopRampPeriod = _ramp;
    motorController->GetConfigurator().Apply(talonFXConfiguration);
}

/**
 * @brief Se configura el limite de corriente del TalonFX
 * @param _currentLimite Limite de corriente del TalonFX
 * @param _enable Se activa el TalonFX
 * @param _triggerThresholdCurrent pendiente...
 * @param _triggerThresholdTime pendiente...
 */
void OverTalonFX::setSupplyCurrentLimit(bool _enable, double _currentLimit, double _triggerThresholdCurrent, double _triggerThresholdTime) {
    talonFXConfiguration.CurrentLimits.SupplyCurrentLimit = _currentLimit;
    talonFXConfiguration.CurrentLimits.SupplyCurrentThreshold = _triggerThresholdCurrent;
    talonFXConfiguration.CurrentLimits.SupplyTimeThreshold = _triggerThresholdTime;
    talonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = _enable;
    motorController->GetConfigurator().Apply(talonFXConfiguration);
}

/**
 * @brief Se configura el limite de torque del TalonFX
 * @param _peakForward Limite de torque del TalonFX(enfrente)
 * @param _peakBackward Limite de torque del TalonFX(atras)
 * @param _deadband Evitar movimientos accidentales y "drift" en controles
 */
void OverTalonFX::setTorqueCurrentLimit(double _peakForward, double _peakBackward, double _deadband) {
    talonFXConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = _peakForward;
    talonFXConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = _peakBackward;
    talonFXConfiguration.TorqueCurrent.TorqueNeutralDeadband = _deadband;
    motorController->GetConfigurator().Apply(talonFXConfiguration);
}

/**
 * @brief Posicion del motor se pone en zero
 */
void OverTalonFX::zeroPosition() {
    motorController->SetRotorPosition(0_tr);
}

/**
 * @brief Se obtiene las revoluciones en metros del TalonFX
 * @param _wheelDiameter Se obtiene el diametro de las llantas
 * @return Revoluciones en metros del TalonFX
 */
double OverTalonFX::getDistance(double _wheelDiameter) {
    motorController->GetRotorPosition().SetUpdateFrequency(50_Hz);
    double sensorPosition = motorController->GetRotorPosition().GetValue().value();
    return(sensorPosition * _wheelDiameter * M_PI) / gearRatio;
}

/**
 * @brief Se obtiene las revoluciones en metros del TalonFX
 * @param _wheelDiameter Se obtiene el diametro de las llantas
 * @return Revoluciones en metros/segundo del TalonFX
 */
double OverTalonFX::getVelocity(double _wheelDiameter) {
    motorController->GetRotorPosition().SetUpdateFrequency(50_Hz);
    double sensorVelocity = motorController->GetRotorVelocity().GetValue().value();
    return(sensorVelocity * _wheelDiameter * M_PI) / gearRatio;
}

/**
 * @brief Se obtiene la posicion absoluta del TalonFX
 * @return Posicion del TalonFX
 */
double OverTalonFX::getPosition() {
    motorController->GetRotorPosition().SetUpdateFrequency(50_Hz);
    return motorController->GetRotorPosition().GetValue().value();
}

/**
 * @brief Se le da voltaje al TalonFX
 * @param _voltage Voltaje que se le da al TalonFX
 * @param enableFOC Activar o desactivar la potencia extra del TalonFX
 */
void OverTalonFX::setVoltage(units::volt_t _voltage, bool enableFOC) {
    VoltageOut voltageOut{ 0_V };
    voltageOut.UpdateFreqHz = 200_Hz;
    voltageOut.EnableFOC = enableFOC;
    motorController->SetControl(voltageOut.WithOutput(_voltage));
}

/**
 * @brief Se obtiene la velocidad mediante el voltaje dado
 * @param _velocity Velocidad que se le da al TalonFX
 * @param enableFOC Activar o desactivar la potencia extra del TalonFX
 */
void OverTalonFX::setVelocityVoltage(double _velocity, bool enableFOC) {
    VelocityVoltage velocityOut{ 0_tps };
    velocityOut.UpdateFreqHz = 200_Hz;
    velocityOut.EnableFOC = enableFOC;
    motorController->SetControl(velocityOut.WithVelocity(units::turns_per_second_t{ _velocity }));
}

/**
 * @brief Se le da el voltaje al TalonFX
 * @param _dutyCycle pendiente....
 * @param enableFOC Activar o desactivar la potencia extra del TalonFX
 */
void OverTalonFX::setDutyCycle(double _dutyCycle, bool enableFOC) {
    DutyCycleOut dutyCycleOut{ 0 };
    dutyCycleOut.UpdateFreqHz = 200_Hz;
    dutyCycleOut.EnableFOC = enableFOC;
    motorController->SetControl(dutyCycleOut.WithOutput(_dutyCycle));
}

/**
 * @brief Se le da una posicion para que el robot llegue(no importa el medio)
 * @param _position La posicion  del robot
 * @param enableFOC Activar o desactivar la potencia extra del TalonFX
 */
void OverTalonFX::setPositionVoltage(double _position, bool enableFOC) {
    PositionVoltage positionVoltage = PositionVoltage{ 0_tr }.WithSlot(0);
    positionVoltage.UpdateFreqHz = 200_Hz;
    positionVoltage.EnableFOC = enableFOC;
    motorController->SetControl(positionVoltage.WithPosition(units::turn_t{ _position }));
}

/**
 * @brief Se le da una posicion para que el robot llegue(mas delicada y con calculos matematicos)
 * @param _position La posicion del robot
 * @param enableFOC Activar o desactivar la potencia extra del TalonFX
 */
void OverTalonFX::setMotionMagicPosition(double _position, bool enableFOC) {
    MotionMagicVoltage motionMagicVoltage = MotionMagicVoltage{ 0_tr }.WithSlot(0);
    motionMagicVoltage.UpdateFreqHz = 200_Hz;
    motionMagicVoltage.EnableFOC = enableFOC;
    motorController->SetControl(motionMagicVoltage.WithPosition(units::turn_t{ _position }));
}

/**
 * @brief
 * @param _velocity Obtener la velocidady
 */
void OverTalonFX::setVelocityTorqueCurrentFOC(double _velocity) {
    VelocityTorqueCurrentFOC velocityTorqueCurrentFOC = VelocityTorqueCurrentFOC{ 0_tps }.WithSlot(0);
    velocityTorqueCurrentFOC.UpdateFreqHz = 200_Hz;
    motorController->SetControl(velocityTorqueCurrentFOC.WithVelocity(units::turns_per_second_t{ _velocity }));
}

/**
 * @brief Se define los valores de PIDValues del TalonFX
 * @param _kP Constante P
 * @param _kI Constante I
 * @param _kD Constante D
 * @param _kS Constante S
 * @param _kV Constante V
 */
void OverTalonFX::setPIDValues(double _kP, double _kI, double _kD, double _kS, double _kV) {
    Slot0Configs slot0Configs{};
    slot0Configs.kP = _kP;
    slot0Configs.kI = _kI;
    slot0Configs.kD = _kD;
    slot0Configs.kS = _kS;
    slot0Configs.kV = _kV;

    motorController->GetConfigurator().Apply(slot0Configs);
}

/**
 * @brief Se configura el Motion Magic(suavizar el recorrido del robot) del TalonFX
 * @param _cruiseVelocity Velocidad del crucero
 * @param _acceleration Acceleracion
 * @param _jerk Inercia
 */
void OverTalonFX::configureMotionMagic(double _cruiseVelocity, double _acceleration, double _jerk) {
    MotionMagicConfigs motionMagicConfigs{};
    motionMagicConfigs.MotionMagicCruiseVelocity = _cruiseVelocity;
    motionMagicConfigs.MotionMagicAcceleration = _acceleration;
    motionMagicConfigs.MotionMagicJerk = _jerk;

    motorController->GetConfigurator().Apply(motionMagicConfigs);
}

/**
 * @brief Se configura el Continuos Wrap del TalonFX
 * @note Revisa la documentacion de phoenix6
 */
void OverTalonFX::setContinuousWrap() {
    talonFXConfiguration.ClosedLoopGeneral.ContinuousWrap = true;
    motorController->GetConfigurator().Apply(talonFXConfiguration);
}

/**
 * @brief Se obtiene el motor Controller del TalonFX
 * @return Motor Controller del TalonFX
 * @note  Se utiliza para poder utilizar las funciones de la libreria de CTRE
 */
TalonFX* OverTalonFX::getMotorController() {
    return(TalonFX*)motorController;
}

