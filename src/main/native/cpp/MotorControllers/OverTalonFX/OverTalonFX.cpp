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

OverTalonFX::OverTalonFX(OverTalonFXConfig overConfig, std::string bus) : TalonFX(
		overConfig.MotorId, bus), overConfig(overConfig) {
	// Configuracion en modo neutral
	setNeutralMode(overConfig.NeutralMode);

	ctreConfig.Voltage.PeakForwardVoltage = 12;
	ctreConfig.Voltage.PeakReverseVoltage = -12;

	// Configuracion modo inverso
	if (overConfig.Inverted) {
		ctreConfig.MotorOutput.Inverted = 1;
	} else {
		ctreConfig.MotorOutput.Inverted = 0;
	}

	// Configuracion de rampas
	ctreConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod =
			overConfig.OpenLoopRampRate.value();
	ctreConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod =
			overConfig.ClosedLoopRampRate.value();

	// Configuracion de corriente
	ctreConfig.CurrentLimits.StatorCurrentLimitEnable = true;
	ctreConfig.CurrentLimits.StatorCurrentLimit =
			overConfig.StatorCurrentLimit.value();

	ctreConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
	ctreConfig.CurrentLimits.SupplyCurrentLimit =
			overConfig.CurrentLimit.value();
	ctreConfig.CurrentLimits.SupplyCurrentThreshold =
			overConfig.TriggerThreshold.value();
	ctreConfig.CurrentLimits.SupplyTimeThreshold =
			overConfig.TriggerThresholdTime.value();

	// Configuracion de PID
	ctreConfig.Slot0.From(overConfig.PIDConfigs);

	// Aplicar la configuracion
	GetConfigurator().Apply(ctreConfig);

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
		ctreConfig.MotorOutput.NeutralMode = 0;
	} else {
		ctreConfig.MotorOutput.NeutralMode = 1;
	}

	GetConfigurator().Apply(ctreConfig);
}

/**
 * @brief Sets the sensor to mechanism ratio of the TalonFX
 *
 * @param gearRatio The gear ratio of the TalonFX
 */
void OverTalonFX::setSensorToMechanism(double gearRatio) {
	ctreConfig.Feedback.SensorToMechanismRatio = gearRatio;
	GetConfigurator().Apply(ctreConfig);
}

/**
 * @brief Sets the rotor to sensor ratio of the TalonFX
 *
 * @param gearRatio The gear ratio of the TalonFX
 */
void OverTalonFX::setRotorToSensorRatio(double gearRatio) {
	ctreConfig.Feedback.RotorToSensorRatio = gearRatio;
	GetConfigurator().Apply(ctreConfig);
}

/**
 * @brief Sets the remote CANCoder of the TalonFX
 *
 * @param deviceID The device ID of the remote CANCoder
 */
void OverTalonFX::setRemoteCANCoder(int id) {
	ctreConfig.Feedback.FeedbackRemoteSensorID = id;
	ctreConfig.Feedback.FeedbackSensorSource =
			FeedbackSensorSourceValue::RemoteCANcoder;
	GetConfigurator().Apply(ctreConfig);
}

/**
 * @brief Sets the fused CANCoder of the TalonFX
 *
 * @param deviceID The device ID of the fused CANCoder
 */
void OverTalonFX::setFusedCANCoder(int id) {
	ctreConfig.Feedback.FeedbackRemoteSensorID = id;
	ctreConfig.Feedback.FeedbackSensorSource =
			FeedbackSensorSourceValue::FusedCANcoder;
	GetConfigurator().Apply(ctreConfig);
}

/**
 * @brief Sets the closed loop torque ramp rate of the TalonFX
 *
 * @param ramp The ramp rate in seconds
 */
void OverTalonFX::setClosedLoopTorqueRamp(units::second_t ramp) {
	ctreConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = ramp.value();
	GetConfigurator().Apply(ctreConfig);
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
	ctreConfig.TorqueCurrent.PeakForwardTorqueCurrent = peakForward.value();
	ctreConfig.TorqueCurrent.PeakReverseTorqueCurrent = peakBackward.value();
	ctreConfig.TorqueCurrent.TorqueNeutralDeadband = deadband.value();
	GetConfigurator().Apply(ctreConfig);
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
const TalonFXConfiguration& OverTalonFX::getCTREConfig() {
	return ctreConfig;
}

/**
 * @brief Sets the TalonFX voltage
 *
 * @param voltage   The voltage to set the TalonFX to
 */
void OverTalonFX::setVoltage(units::volt_t voltage) {
	SetControl(VoltageOut { voltage }.WithEnableFOC(overConfig.useFOC));
}

/**
 * @brief Sets the TalonFX Velocity using Voltage
 *
 * @param velocity  The velocity to set the TalonFX to
 */
void OverTalonFX::setVelocityVoltage(units::turns_per_second_t velocity) {
	SetControl(VelocityVoltage { velocity }.WithEnableFOC(overConfig.useFOC));
}

/**
 * @brief Sets the TalonFX Duty Cycle
 *
 * @param dutyCycle The duty cycle from -1 to 1
 */
void OverTalonFX::setDutyCycle(units::scalar_t dutyCycle) {
	SetControl(DutyCycleOut { dutyCycle }.WithEnableFOC(overConfig.useFOC));
}

/**
 * @brief Sets the TalonFX Position using Voltage
 *
 * @param position  The position to set the TalonFX to
 */
void OverTalonFX::setPositionVoltage(units::turn_t position) {
	SetControl(PositionVoltage { position }.WithEnableFOC(overConfig.useFOC));
}

/**
 * @brief Sets the TalonFX Motion Magic Position using Voltage
 *
 * @param position    The position to set the TalonFX to
 * @param feedForward The feed forward to set the TalonFX to
 */
void OverTalonFX::setMotionMagicPosition(units::turn_t position) {
	SetControl(
			MotionMagicVoltage { position }.WithEnableFOC(overConfig.useFOC));
}

/**
 * @brief Sets the TalonFX Velocity using Torque FOC
 *
 * @param velocity The velocity to set the TalonFX to
 */
void OverTalonFX::setVelocityTorqueCurrentFOC(
		units::turns_per_second_t velocity) {
	SetControl(VelocityTorqueCurrentFOC { velocity });
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
	ctreConfig.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity.value();
	ctreConfig.MotionMagic.MotionMagicAcceleration = acceleration.value();
	ctreConfig.MotionMagic.MotionMagicJerk = jerk.value();

	GetConfigurator().Apply(ctreConfig);
}

/**
 * @brief Configures the TalonFX software limit switch
 *
 * @param configs The software limit switch configurations
 */
void OverTalonFX::configureSoftwareLimitSwitch(
		ctre::phoenix6::configs::SoftwareLimitSwitchConfigs configs) {
	ctreConfig.SoftwareLimitSwitch = configs;
	GetConfigurator().Apply(ctreConfig);
}

/**
 * @brief Sets the TalonFX continuous wrap
 */
void OverTalonFX::setContinuousWrap() {
	ctreConfig.ClosedLoopGeneral.ContinuousWrap = true;
	GetConfigurator().Apply(ctreConfig);
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
