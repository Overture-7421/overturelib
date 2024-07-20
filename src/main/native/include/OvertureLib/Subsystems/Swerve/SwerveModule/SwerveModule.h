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

class SwerveModule: public frc2::SubsystemBase {
public:
	SwerveModule(int rotatorID, int wheelID, int canCoderID,
			units::turn_t offSet, std::string moduleName, std::string canBus);
	void setRotatorPIDValues(double kP, double kI, double kD);
	void setDrivePIDValues(double kP, double kI, double kD);
	void setFFConstants(units::volt_t ks, units::volt_t kv, units::volt_t ka);
	void setGearRatio(double _turn, double _wheel);
	void setWheelDiameter(double _wheelDiameter);

	double getSpeed();
	double setSpeed(double speed);
	double getDistance();
	double getAngle();
	double getVoltage();

	frc::SwerveModuleState getState();
	void setState(frc::SwerveModuleState state);
	frc::SwerveModulePosition getPosition();

	void setRawVoltageSpeed(units::volt_t volts);
	void setVoltages();

	void Periodic() override;

private:
	//Declaration of motor controllers
	std::unique_ptr<OverTalonFX> m_driveMotor;
	std::unique_ptr<OverTalonFX> m_turningMotor;

	//Declaration of sensors
	std::unique_ptr<OverCANCoder> m_canCoder;

	//FeedForward
	std::shared_ptr<frc::SimpleMotorFeedforward<units::meters>> m_feedForward;

	//State
	frc::SwerveModuleState m_state;

	//Gear and Wheel
	double m_wheelDiameter = 1;

	std::string m_name;
	bool useRawVoltageSpeed = false;
};
