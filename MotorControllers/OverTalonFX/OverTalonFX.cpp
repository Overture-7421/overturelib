// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "OverTalonFX.h"

/**
 * @brief Constructor for OverTalonFX
 *
 * @param id          The ID of the TalonFX
 * @param inverted    Whether or not the TalonFX is inverted
 * @param neutralMode The neutral mode of the TalonFX
 * @param bus         The bus of the TalonFX
 */

OverTalonFX::OverTalonFX(int id, ControllerNeutralMode neutralMode, bool inverted, std::string bus) : TalonFX(id, bus) {
	// Configuracion en modo neutral
	setNeutralMode(neutralMode);

	config.Voltage.PeakForwardVoltage = 12;
	config.Voltage.PeakReverseVoltage = -12;

	// Configuracion modo inverso
	if (inverted) {
		config.MotorOutput.Inverted = 1;
	} else {
		config.MotorOutput.Inverted = 0;
	}

	// Aplicar la configuracion
	GetConfigurator().Apply(config);
}

/**
 * Sets the neutral mode of the TalonFX
 *
 * @param neutralMode The neutral mode of the TalonFX
 */
void OverTalonFX::setNeutralMode(ControllerNeutralMode neutralMode) {
	// Cambiar el modo neutral
	if (neutralMode == ControllerNeutralMode::Coast) {
		config.MotorOutput.NeutralMode = 0;
	} else {
		config.MotorOutput.NeutralMode = 1;
	}

	GetConfigurator().Apply(config);
}

/**
 * @brief Sets the sensor to mechanism ratio of the TalonFX
 *
 * @param gearRatio The gear ratio of the TalonFX
 */
void OverTalonFX::setSensorToMechanism(double gearRatio) {
	config.Feedback.SensorToMechanismRatio = gearRatio;
	GetConfigurator().Apply(config);
}

/**
 * @brief Sets the rotor to sensor ratio of the TalonFX
 *
 * @param gearRatio The gear ratio of the TalonFX
*/
void OverTalonFX::setRotorToSensorRatio(double gearRatio) {
	config.Feedback.RotorToSensorRatio = gearRatio;
	GetConfigurator().Apply(config);
}

/**
 * @brief Sets the remote CANCoder of the TalonFX
 *
 * @param deviceID The device ID of the remote CANCoder
 */
void OverTalonFX::setRemoteCANCoder(int id) {
	config.Feedback.FeedbackRemoteSensorID = id;
	config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue::RemoteCANcoder;
	GetConfigurator().Apply(config);
}

/**
 * @brief Sets the fused CANCoder of the TalonFX
 *
 * @param deviceID The device ID of the fused CANCoder
 */
void OverTalonFX::setFusedCANCoder(int id) {
	config.Feedback.FeedbackRemoteSensorID = id;
	config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue::FusedCANcoder;
	GetConfigurator().Apply(config);
}

/**
 * @brief Sets the closed loop voltage ramp rate of the TalonFX
 *
 * @param ramp The closed loop voltage ramp rate of the TalonFX
 */
void OverTalonFX::setClosedLoopVoltageRamp(double ramp) {
	config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = ramp;
	GetConfigurator().Apply(config);
}

/**
 * @brief Sets the closed loop torque ramp rate of the TalonFX
 *
 * @param ramp The closed loop torque ramp rate of the TalonFX
 */
void OverTalonFX::setClosedLoopTorqueRamp(double ramp) {
	config.ClosedLoopRamps.TorqueClosedLoopRampPeriod = ramp;
	GetConfigurator().Apply(config);
}

/**
 * @brief Sets the supply current limit of the TalonFX
 *
 * @param enable                  Whether or not the supply current limit is enabled
 * @param currentLimit            The supply current limit of the TalonFX
 * @param triggerThresholdCurrent The trigger threshold current of the TalonFX
 * @param triggerThresholdTime    The trigger threshold time of the TalonFX
 */
void OverTalonFX::setSupplyCurrentLimit(bool enable, double currentLimit, double triggerThresholdCurrent, double triggerThresholdTime) {
	config.CurrentLimits.SupplyCurrentLimitEnable = enable;
	config.CurrentLimits.SupplyCurrentLimit = currentLimit;
	config.CurrentLimits.SupplyCurrentThreshold = triggerThresholdCurrent;
	config.CurrentLimits.SupplyTimeThreshold = triggerThresholdTime;
	GetConfigurator().Apply(config);
}

/**
 * @brief Sets the torque current limit of the TalonFX
 *
 * @param peakForwardVoltage The peak forward voltage of the TalonFX
 * @param peakReverseVoltage The peak reverse voltage of the TalonFX
 * @param deadband           The deadband of the TalonFX
 */
void OverTalonFX::setTorqueCurrentLimit(double peakForward, double peakBackward, double deadband) {
	config.TorqueCurrent.PeakForwardTorqueCurrent = peakForward;
	config.TorqueCurrent.PeakReverseTorqueCurrent = peakBackward;
	config.TorqueCurrent.TorqueNeutralDeadband = deadband;
	GetConfigurator().Apply(config);
}

/**
 * @brief Sets the TalonFX to follow another TalonFX
 *
 * @param masterID The ID of the TalonFX to follow
 * @param inverted Whether or not the TalonFX is inverted
 */
void OverTalonFX::setFollow(int masterID, bool inverted) {
	SetControl(Follower{ masterID, inverted });
}

/**
 * @brief Sets the TalonFX position to zero
 */
void OverTalonFX::zeroPosition() {
	SetPosition(0_tr);
}

/**
 * @brief Sets the TalonFX position
 *
 * @param position The position to set the TalonFX to
 */
void OverTalonFX::setSensorPosition(double position) {
	SetPosition(units::turn_t{ position });
}


/**
 * @brief Gets the TalonFX position in meters
 *
 * @param wheelDiameter The diameter of the wheel
 * @param gearRatio     The gear ratio of the TalonFX
 * @return The TalonFX position in meters
 */
double OverTalonFX::getDistance(double wheelDiameter, double gearRatio) {
	double sensorPosition = GetRotorPosition().GetValue().value();
	return(sensorPosition * wheelDiameter * M_PI) / gearRatio;
}

/**
 * @brief Gets the TalonFX position in meters
 *
 * @param wheelDiameter The diameter of the wheel
 * @return The TalonFX position in meters
 */
double OverTalonFX::getDistance(double wheelDiameter) {
	double sensorPosition = GetPosition().GetValue().value();
	return(sensorPosition * wheelDiameter * M_PI);
}

/**
 * @brief Gets the TalonFX velocity in meters per second
 *
 * @param wheelDiameter The diameter of the wheel
 * @param gearRatio     The gear ratio of the TalonFX
 */
double OverTalonFX::getVelocity(double _wheelDiameter, double gearRatio) {
	double sensorVelocity = GetRotorVelocity().GetValue().value();
	return(sensorVelocity * _wheelDiameter * M_PI) / gearRatio;
}

/**
 * @brief Gets the TalonFX velocity in meters per second
 *
 * @param wheelDiameter The diameter of the wheel
 */
double OverTalonFX::getVelocity(double _wheelDiameter) {
	double sensorVelocity = GetVelocity().GetValue().value();
	return(sensorVelocity * _wheelDiameter * M_PI);
}

/**
 * @brief Gets the TalonFX absolute position
 *
 * @return The TalonFX absolute position
 */
double OverTalonFX::getPosition() {
	return GetPosition().GetValue().value();
}

/**
 * @brief Sets the TalonFX voltage
 *
 * @param voltage   The voltage to set the TalonFX to
 * @param enableFOC Whether or not to enable FOC
 */
void OverTalonFX::setVoltage(units::volt_t voltage, bool enableFOC) {
	VoltageOut voltageOut{ 0_V };
	voltageOut.EnableFOC = enableFOC;
	SetControl(voltageOut.WithOutput(voltage));
}

/**
 * @brief Sets the TalonFX Velocity using Voltage
 *
 * @param velocity  The velocity to set the TalonFX to
 * @param enableFOC Whether or not to enable FOC
 */
void OverTalonFX::setVelocityVoltage(double velocity, bool enableFOC) {
	VelocityVoltage velocityOut{ 0_tps };
	velocityOut.EnableFOC = enableFOC;
	SetControl(velocityOut.WithVelocity(units::turns_per_second_t{ velocity }));
}

/**
 * @brief Sets the TalonFX Duty Cycle
 *
 * @param dutyCycle The duty cycle to set the TalonFX to
 * @param enableFOC Whether or not to enable FOC
 */
void OverTalonFX::setDutyCycle(double dutyCycle, bool enableFOC) {
	DutyCycleOut dutyCycleOut{ 0 };
	dutyCycleOut.EnableFOC = enableFOC;
	SetControl(dutyCycleOut.WithOutput(dutyCycle));
}

/**
 * @brief Sets the TalonFX Position using Voltage
 *
 * @param position  The position to set the TalonFX to
 * @param enableFOC Whether or not to enable FOC
 */
void OverTalonFX::setPositionVoltage(double position, bool enableFOC) {
	PositionVoltage positionVoltage = PositionVoltage{ 0_tr }.WithSlot(0);
	positionVoltage.EnableFOC = enableFOC;
	SetControl(positionVoltage.WithPosition(units::turn_t{ position }));
}

/**
 * @brief Sets the TalonFX Motion Magic Position using Voltage
 *
 * @param position    The position to set the TalonFX to
 * @param feedForward The feed forward to set the TalonFX to
 * @param enableFOC   Whether or not to enable FOC
 */
void OverTalonFX::setMotionMagicPosition(double position, double feedForward, bool enableFOC) {
	MotionMagicVoltage motionMagicVoltage = MotionMagicVoltage{ 0_tr }.WithSlot(0);
	motionMagicVoltage.FeedForward = units::volt_t{ feedForward };
	motionMagicVoltage.EnableFOC = enableFOC;
	SetControl(motionMagicVoltage.WithPosition(units::turn_t{ position }));
}

/**
 * @brief Sets the TalonFX Velocity using Torque FOC
 *
 * @param velocity The velocity to set the TalonFX to
 */
void OverTalonFX::setVelocityTorqueCurrentFOC(double velocity) {
	VelocityTorqueCurrentFOC velocityTorqueCurrentFOC = VelocityTorqueCurrentFOC{ 0_tps }.WithSlot(0);
	SetControl(velocityTorqueCurrentFOC.WithVelocity(units::turns_per_second_t{ velocity }));
}

/**
 * @brief Sets the TalonFX PID values
 *
 * @param kP The P value of the TalonFX
 * @param kI The I value of the TalonFX
 * @param kD The D value of the TalonFX
 * @param kS The S value of the TalonFX
 * @param kV The V value of the TalonFX
 */
void OverTalonFX::setPIDValues(double kP, double kI, double kD, double kS, double kV) {
	Slot0Configs slot0Configs{};
	slot0Configs.kP = kP;
	slot0Configs.kI = kI;
	slot0Configs.kD = kD;
	slot0Configs.kS = kS;
	slot0Configs.kV = kV;

	GetConfigurator().Apply(slot0Configs);
}

/**
 * @brief Sets the TalonFX Motion Magic values
 *
 * @param cruiseVelocity The cruise velocity of the TalonFX
 * @param acceleration   The acceleration of the TalonFX
 * @param jerk           The jerk of the TalonFX
 */
void OverTalonFX::configureMotionMagic(double cruiseVelocity, double acceleration, double jerk) {
	MotionMagicConfigs motionMagicConfigs{};
	motionMagicConfigs.MotionMagicCruiseVelocity = cruiseVelocity;
	motionMagicConfigs.MotionMagicAcceleration = acceleration;
	motionMagicConfigs.MotionMagicJerk = jerk;

	GetConfigurator().Apply(motionMagicConfigs);
}

/**
 * @brief Sets the TalonFX continuous wrap
 */
void OverTalonFX::setContinuousWrap() {
	config.ClosedLoopGeneral.ContinuousWrap = true;
	GetConfigurator().Apply(config);
}
