// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once 

#include "OvertureLib/motorControllers/overController/overController.h"

#include <math.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>
#include <ctre/phoenix6/signals/SpnEnums.hpp>

#include <frc/smartdashboard/SmartDashboard.h>

using namespace ctre::phoenix6::hardware;
using namespace ctre::phoenix6::configs;
using namespace ctre::phoenix6::signals;
using namespace ctre::phoenix6::controls;

class OverTalonFX: public OverController<TalonFX> {
public:
  OverTalonFX(int _id, ControllerNeutralMode _neutralMode, bool _inverted, double _gearRatio, std::string _bus);


  void init();
  void reset();

  void setNeutralMode(ControllerNeutralMode _neutralMode);
  void setSensorToMechanism(double _gearRatio);
  void setRotorToSensorRatio(double _gearRatio);
  void setRemoteCANCoder(int _id);
  void setFusedCANCoder(int _id);
  void setClosedLoopVoltageRamp(double _ramp);
  void setClosedLoopTorqueRamp(double _ramp);
  void setSupplyCurrentLimit(bool _enable, double _currentLimit, double _triggerThresholdCurrent, double _triggerThresholdTime);
  void setTorqueCurrentLimit(double peakForward, double peakBackward, double deadband);
  void zeroPosition();

  double getDistance(double _wheelDiameter);
  double getVelocity(double _wheelDiameter);
  double getPosition();

  void setVoltage(units::volt_t _voltage, bool enableFOC);
  void setVelocityVoltage(double _velocity, bool enableFOC);
  void setDutyCycle(double _dutyCycle, bool enableFOC);
  void setPositionVoltage(double _position, bool enableFOC);
  void setMotionMagicPosition(double _position, bool enableFOC);
  void setVelocityTorqueCurrentFOC(double _velocity);

  void setPIDValues(double _kP, double _kI, double _kD, double _kS, double _kV);
  void configureMotionMagic(double _cruiseVelocity, double _acceleration, double _jerk);
  void setContinuousWrap();

  TalonFX* getMotorController();

private:
  TalonFXConfiguration talonFXConfiguration;
};
