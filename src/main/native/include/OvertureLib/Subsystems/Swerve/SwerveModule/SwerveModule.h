// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <memory.h>

#include "OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h"
#include "OvertureLib/Sensors/OverCANCoder/OverCANCoder.h"
#include "ModuleConfig.h"

class SwerveModule: public frc2::SubsystemBase {
public:
	SwerveModule(SwerveModuleConfig config);

	const frc::SwerveModuleState& getState();
	void setState(frc::SwerveModuleState state);
	const frc::SwerveModulePosition& getPosition();

	void setVoltageDrive(units::volt_t volts);
	units::volt_t getVoltageDrive();

	void shuffleboardPeriodic();
	void Periodic() override;

private:
	SwerveModuleConfig config;
	//Declaration of motor controllers
	OverTalonFX driveMotor;
	OverTalonFX turnMotor;

	//Declaration of sensors
	OverCANCoder canCoder;

	//FeedForward
	frc::SimpleMotorFeedforward<units::meters> feedForward;

	//State
	frc::SwerveModuleState targetState;

	frc::SwerveModuleState latestState;
	frc::SwerveModulePosition latestPosition;

	bool useRawVoltageDrive = false;

	PositionVoltage turnVoltage { 0_tr };
	VoltageOut driveVoltage { 0_V };
};
