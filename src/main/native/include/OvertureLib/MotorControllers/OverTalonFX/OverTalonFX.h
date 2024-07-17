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

class OverTalonFX : public TalonFX {
public:
	OverTalonFX(int id, ControllerNeutralMode neutralMode, bool inverted, std::string bus);

	void setNeutralMode(ControllerNeutralMode neutralMode);
	void setSensorToMechanism(double gearRatio);
	void setRotorToSensorRatio(double gearRatio);
	void setRemoteCANCoder(int id);
	void setFusedCANCoder(int id);
	void setClosedLoopVoltageRamp(double ramp);
	void setClosedLoopTorqueRamp(double ramp);
	void setSupplyCurrentLimit(bool enable, double currentLimit, double triggerThresholdCurrent, double triggerThresholdTime);
	void setTorqueCurrentLimit(double peakForward, double peakBackward, double deadband);
	void setFollow(int masterID, bool inverted);
	void zeroPosition();
	void setSensorPosition(double position);

	double getDistance(double wheelDiameter, double gearRatio);
	double getDistance(double wheelDiameter);
	double getVelocity(double wheelDiameter, double gearRatio);
	double getVelocity(double wheelDiameter);
	double getPosition();
	const TalonFXConfiguration& getConfig();

	void setVoltage(units::volt_t voltage, bool enableFOC);
	void setVelocityVoltage(double velocity, double feedForward, bool enableFOC);
	void setDutyCycle(double dutyCycle, bool enableFOC);
	void setPositionVoltage(double position, bool enableFOC);
	void setMotionMagicPosition(double position, double feedForward, bool enableFOC);
	void setVelocityTorqueCurrentFOC(double velocity);

	void setPIDValues(double kP, double kI, double kD, double kS, double kV);
	void configureMotionMagic(double cruiseVelocity, double acceleration, double jerk);
	void configureSoftwareLimitSwitch(ctre::phoenix6::configs::SoftwareLimitSwitchConfigs configs);
	void setContinuousWrap();

	void setPositionUpdateFrequency(units::frequency::hertz_t frequencyHz);
	void setVelocityUpdateFrequency(units::frequency::hertz_t frequencyHz);

private:
	TalonFXConfiguration config;
};