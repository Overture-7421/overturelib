// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h"

#ifndef __FRC_ROBORIO__
#include "OvertureLib/Simulation/SimMotorManager/SimMotorManager.h"
#endif

/**
 * @brief Constructor for OverTalonFX
 *
 * @param id          The ID of the TalonFX
 * @param inverted    Whether or not the TalonFX is inverted
 * @param neutralMode The neutral mode of the TalonFX
 * @param bus         The bus of the TalonFX
 */

OverTalonFX::OverTalonFX(int id, ControllerNeutralMode neutralMode,
		bool inverted, std::string bus) : TalonFX(id, bus) {
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

#ifndef __FRC_ROBORIO__
	SimMotorManager &simMotorManager = SimMotorManager::GetInstance();
	simMotorManager.AddSimMotorCandidate(this);
#endif
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
	config.Feedback.FeedbackSensorSource =
			FeedbackSensorSourceValue::RemoteCANcoder;
	GetConfigurator().Apply(config);
}

/**
 * @brief Sets the fused CANCoder of the TalonFX
 *
 * @param deviceID The device ID of the fused CANCoder
 */
void OverTalonFX::setFusedCANCoder(int id) {
	config.Feedback.FeedbackRemoteSensorID = id;
	config.Feedback.FeedbackSensorSource =
			FeedbackSensorSourceValue::FusedCANcoder;
	GetConfigurator().Apply(config);
}

/**
 * @brief Sets the open loop voltage ramp rate of the TalonFX
 *
 * @param ramp The ramp rate in seconds
 */
void OverTalonFX::setOpenLoopVoltageRamp(units::second_t ramp) {
	config.OpenLoopRamps.VoltageOpenLoopRampPeriod = ramp.value();
	GetConfigurator().Apply(config);
}

/**
 * @brief Sets the closed loop voltage ramp rate of the TalonFX
 *
 * @param ramp The ramp rate in seconds
 */
void OverTalonFX::setClosedLoopVoltageRamp(units::second_t ramp) {
	config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = ramp.value();
	GetConfigurator().Apply(config);
}

/**
 * @brief Sets the closed loop torque ramp rate of the TalonFX
 *
 * @param ramp The ramp rate in seconds
 */
void OverTalonFX::setClosedLoopTorqueRamp(units::second_t ramp) {
	config.ClosedLoopRamps.TorqueClosedLoopRampPeriod = ramp.value();
	GetConfigurator().Apply(config);
}

/**
 * @brief Sets the stator current limit of the TalonFX
 *
 * @param enable Whether or not the stator current limit is enabled
 * @param currentLimit The stator current limit of the TalonFX
 */
void OverTalonFX::setStatorCurrentLimit(bool enable,
		units::ampere_t currentLimit) {
	config.CurrentLimits.StatorCurrentLimitEnable = enable;
	config.CurrentLimits.StatorCurrentLimit = currentLimit.value();
	GetConfigurator().Apply(config);
}

/**
 * @brief Sets the supply current limit of the TalonFX
 *
 * @param enable                  Whether or not the supply current limit is enabled
 * @param currentLimit            The supply current limit in Amps
 * @param triggerThresholdCurrent The trigger threshold in Amps
 * @param triggerThresholdTime    The trigger threshold time in seconds
 */
void OverTalonFX::setSupplyCurrentLimit(bool enable,
		units::ampere_t currentLimit, units::ampere_t triggerThresholdCurrent,
		units::second_t triggerThresholdTime) {
	config.CurrentLimits.SupplyCurrentLimitEnable = enable;
	config.CurrentLimits.SupplyCurrentLimit = currentLimit.value();
	config.CurrentLimits.SupplyCurrentThreshold =
			triggerThresholdCurrent.value();
	config.CurrentLimits.SupplyTimeThreshold = triggerThresholdTime.value();
	GetConfigurator().Apply(config);
}

/**
 * @brief Sets the torque current limit of the TalonFX
 *
 * @param peakForwardVoltage The peak forward voltage of the TalonFX
 * @param peakReverseVoltage The peak reverse voltage of the TalonFX
 * @param deadband           The deadband of the TalonFX
 */
void OverTalonFX::setTorqueCurrentLimit(units::ampere_t peakForward,
		units::ampere_t peakBackward, units::ampere_t deadband) {
	config.TorqueCurrent.PeakForwardTorqueCurrent = peakForward.value();
	config.TorqueCurrent.PeakReverseTorqueCurrent = peakBackward.value();
	config.TorqueCurrent.TorqueNeutralDeadband = deadband.value();
	GetConfigurator().Apply(config);
}

/**
 * @brief Sets the TalonFX to follow another TalonFX
 *
 * @param masterID The ID of the TalonFX to follow
 * @param inverted Whether or not the TalonFX is inverted
 */
void OverTalonFX::setFollow(int masterID, bool inverted) {
	SetControl(Follower { masterID, inverted });
}

/**
 * @brief Gets the TalonFX configuration
 *
 * @return The TalonFX configuration
 */
const TalonFXConfiguration& OverTalonFX::getConfig() {
	return config;
}

/**
 * @brief Sets the TalonFX voltage
 *
 * @param voltage   The voltage to set the TalonFX to
 * @param enableFOC Whether or not to enable FOC
 */
void OverTalonFX::setVoltage(units::volt_t voltage, bool enableFOC) {
	VoltageOut voltageOut { 0_V };
	voltageOut.EnableFOC = enableFOC;
	SetControl(voltageOut.WithOutput(voltage));
}

/**
 * @brief Sets the TalonFX Velocity using Voltage
 *
 * @param velocity  The velocity to set the TalonFX to
 * @param enableFOC Whether or not to enable FOC
 */
void OverTalonFX::setVelocityVoltage(units::turns_per_second_t velocity,
		units::volt_t feedForward, bool enableFOC) {
	VelocityVoltage velocityOut { 0_tps };
	velocityOut.EnableFOC = enableFOC;
	velocityOut.FeedForward = feedForward;
	SetControl(velocityOut.WithVelocity(velocity));
}

/**
 * @brief Sets the TalonFX Duty Cycle
 *
 * @param dutyCycle The duty cycle from -1 to 1
 * @param enableFOC Whether or not to enable FOC
 */
void OverTalonFX::setDutyCycle(units::scalar_t dutyCycle, bool enableFOC) {
	DutyCycleOut dutyCycleOut { 0 };
	dutyCycleOut.EnableFOC = enableFOC;
	SetControl(dutyCycleOut.WithOutput(dutyCycle));
}

/**
 * @brief Sets the TalonFX Position using Voltage
 *
 * @param position  The position to set the TalonFX to
 * @param enableFOC Whether or not to enable FOC
 */
void OverTalonFX::setPositionVoltage(units::turn_t position, bool enableFOC) {
	PositionVoltage positionVoltage = PositionVoltage { 0_tr }.WithSlot(0);
	positionVoltage.EnableFOC = enableFOC;
	SetControl(positionVoltage.WithPosition(position));
}

/**
 * @brief Sets the TalonFX Motion Magic Position using Voltage
 *
 * @param position    The position to set the TalonFX to
 * @param feedForward The feed forward to set the TalonFX to
 * @param enableFOC   Whether or not to enable FOC
 */
void OverTalonFX::setMotionMagicPosition(units::turn_t position,
		units::volt_t feedForward, bool enableFOC) {
	MotionMagicVoltage motionMagicVoltage =
			MotionMagicVoltage { 0_tr }.WithSlot(0);
	motionMagicVoltage.FeedForward = feedForward;
	motionMagicVoltage.EnableFOC = enableFOC;
	SetControl(motionMagicVoltage.WithPosition(position));
}

/**
 * @brief Sets the TalonFX Velocity using Torque FOC
 *
 * @param velocity The velocity to set the TalonFX to
 */
void OverTalonFX::setVelocityTorqueCurrentFOC(
		units::turns_per_second_t velocity) {
	VelocityTorqueCurrentFOC velocityTorqueCurrentFOC =
			VelocityTorqueCurrentFOC { 0_tps }.WithSlot(0);
	SetControl(velocityTorqueCurrentFOC.WithVelocity(velocity));
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
void OverTalonFX::setPIDValues(double kP, double kI, double kD, double kS,
		double kV) {
	config.Slot0.kP = kP;
	config.Slot0.kI = kI;
	config.Slot0.kD = kD;
	config.Slot0.kS = kS;
	config.Slot0.kV = kV;

	GetConfigurator().Apply(config);
}

/**
 * @brief Sets the TalonFX Motion Magic values
 *
 * @param cruiseVelocity The cruise velocity of the TalonFX
 * @param acceleration   The acceleration of the TalonFX
 * @param jerk           The jerk of the TalonFX
 */
void OverTalonFX::configureMotionMagic(units::turns_per_second_t cruiseVelocity,
		units::turns_per_second_squared_t acceleration,
		units::turns_per_second_cubed_t jerk) {
	config.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity.value();
	config.MotionMagic.MotionMagicAcceleration = acceleration.value();
	config.MotionMagic.MotionMagicJerk = jerk.value();

	GetConfigurator().Apply(config);
}

/**
 * @brief Configures the TalonFX software limit switch
 *
 * @param configs The software limit switch configurations
 */
void OverTalonFX::configureSoftwareLimitSwitch(
		ctre::phoenix6::configs::SoftwareLimitSwitchConfigs configs) {
	config.SoftwareLimitSwitch = configs;
	GetConfigurator().Apply(config);
}

/**
 * @brief Sets the TalonFX continuous wrap
 */
void OverTalonFX::setContinuousWrap() {
	config.ClosedLoopGeneral.ContinuousWrap = true;
	GetConfigurator().Apply(config);
}

/**
 * @brief Sets the TalonFX position update frequency
 *
 * @param frequencyHz The frequency to set the TalonFX to
 */
void OverTalonFX::setPositionUpdateFrequency(
		units::frequency::hertz_t frequencyHz) {
	GetPosition().SetUpdateFrequency(frequencyHz);
}

/**
 * @brief Sets the TalonFX velocity update frequency
 *
 * @param frequencyHz The frequency to set the TalonFX to
 */
void OverTalonFX::setVelocityUpdateFrequency(
		units::frequency::hertz_t frequencyHz) {
	GetVelocity().SetUpdateFrequency(frequencyHz);
}
