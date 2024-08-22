// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once 

#include "OvertureLib/MotorControllers/ControllerNeutralMode/ControllerNeutralMode.h"

#include <math.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>
#include <ctre/phoenix6/signals/SpnEnums.hpp>

#include <frc/smartdashboard/SmartDashboard.h>

using namespace ctre::phoenix6::hardware;
using namespace ctre::phoenix6::configs;
using namespace ctre::phoenix6::signals;
using namespace ctre::phoenix6::controls;

class OverTalonFX: public TalonFX {
public:
	OverTalonFX(int id, ControllerNeutralMode neutralMode, bool inverted,
			std::string bus);

	void setNeutralMode(ControllerNeutralMode neutralMode);
	void setSensorToMechanism(double gearRatio);
	void setRotorToSensorRatio(double gearRatio);
	void setRemoteCANCoder(int id);
	void setFusedCANCoder(int id);
	void setOpenLoopVoltageRamp(units::second_t ramp);
	void setClosedLoopVoltageRamp(units::second_t ramp);
	void setClosedLoopTorqueRamp(units::second_t ramp);
	void setStatorCurrentLimit(bool enable, units::ampere_t currentLimit);
	void setSupplyCurrentLimit(bool enable, units::ampere_t currentLimit,
			units::ampere_t triggerThresholdCurrent,
			units::second_t triggerThresholdTime);
	void setTorqueCurrentLimit(units::ampere_t peakForward,
			units::ampere_t peakBackward, units::ampere_t deadband);
	void setFollow(int masterID, bool inverted);
	const TalonFXConfiguration& getConfig();

	void setVoltage(units::volt_t voltage, bool enableFOC);
	void setVelocityVoltage(units::turns_per_second_t velocity,
			units::volt_t feedForward, bool enableFOC);
	void setDutyCycle(units::scalar_t dutyCycle, bool enableFOC);
	void setPositionVoltage(units::turn_t position, bool enableFOC);
	void setMotionMagicPosition(units::turn_t position,
			units::volt_t feedForward, bool enableFOC);
	void setVelocityTorqueCurrentFOC(units::turns_per_second_t velocity);

	void setPIDValues(double kP, double kI, double kD, double kS, double kV);
	void configureMotionMagic(units::turns_per_second_t cruiseVelocity,
			units::turns_per_second_squared_t acceleration,
			units::turns_per_second_cubed_t jerk);
	void configureSoftwareLimitSwitch(
			ctre::phoenix6::configs::SoftwareLimitSwitchConfigs configs);
	void setContinuousWrap();

	void setPositionUpdateFrequency(units::frequency::hertz_t frequencyHz);
	void setVelocityUpdateFrequency(units::frequency::hertz_t frequencyHz);

private:
	TalonFXConfiguration config;
};
