// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once 

#include "OvertureLib/MotorControllers/ControllerNeutralMode/ControllerNeutralMode.h"
#include "OvertureLib/MotorControllers/OverTalonFX/Config.h"

#include <math.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/signals/SpnEnums.hpp>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Alert.h>

using namespace ctre::phoenix6::hardware;
using namespace ctre::phoenix6::configs;
using namespace ctre::phoenix6::signals;
using namespace ctre::phoenix6::controls;

class OverTalonFX: public TalonFX {
public:
	OverTalonFX(OverTalonFXConfig overConfig, std::string bus);
	void updateAlert();
	void setSensorToMechanism(double gearRatio);
	void setRotorToSensorRatio(double gearRatio);
	void setRemoteCANCoder(int id);
	void setFusedCANCoder(int id);
	void setClosedLoopTorqueRamp(units::second_t ramp);
	void setTorqueCurrentLimit(units::ampere_t peakForward,
			units::ampere_t peakBackward, units::ampere_t deadband);
	void setFollow(int masterID, bool inverted);
	const TalonFXConfiguration& getCTREConfig();
	void configureMotionMagic(units::turns_per_second_t cruiseVelocity,
			units::turns_per_second_squared_t acceleration,
			units::turns_per_second_cubed_t jerk);
	void configureSoftwareLimitSwitch(
			ctre::phoenix6::configs::SoftwareLimitSwitchConfigs configs);
	void setContinuousWrap();

	void setPositionUpdateFrequency(units::frequency::hertz_t frequencyHz);
	void setVelocityUpdateFrequency(units::frequency::hertz_t frequencyHz);

private:
	TalonFXConfiguration ctreConfig;
	OverTalonFXConfig overConfig;

	frc::Alert isConnectedAlert { "Devices", "TalonFX is not connected",
			frc::Alert::AlertType::kError };
};
