// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "OvertureLib/MotorControllers/OverTalonFX/OverTalonFX.h"
#include "OvertureLib/Sensors/OverCANCoder/OverCANCoder.h"

class SwerveModule : public frc2::SubsystemBase {
public:
	SwerveModule(int rotatorID, int wheelID, int canCoderID, double offSet, std::string moduleName, std::string canBus);
	void setRotatorPIDValues(double kP, double kI, double kD);
	void setDrivePIDValues(double kP, double kI, double kD);
	void setFFConstants(units::volt_t ks, units::volt_t kv, units::volt_t ka);
	void setGearRatio(double _turn, double _wheel);
	void setWheelDiameter(double _wheelDiameter);

	double getSpeed();
	double setSpeed(double speed);
	double getDistance();
	double getAngle();

	frc::SwerveModuleState getState();
	void setState(frc::SwerveModuleState state);
	frc::SwerveModulePosition getPosition();

	void setVoltages();

	void Periodic() override;

private:
	//Declaration of motor controllers
	OverTalonFX* m_driveMotor;
	OverTalonFX* m_turningMotor;

	//Declaration of sensors
	OverCANCoder* m_canCoder;

	//FeedForward
	frc::SimpleMotorFeedforward<units::meters> m_feedForward;

	//State
	frc::SwerveModuleState m_state;

	//Gear and Wheel
	double m_wheelDiameter = 1;

	std::string m_name;
	bool useRawVoltageSpeed = false;
};