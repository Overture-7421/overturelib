// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/geometry/Rotation2d.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/Trigger.h>
#include <frc2/command/button/CommandGenericHID.h>
#include <frc2/command/button/CommandXboxController.h>
#include <Math/Utils.h>

#pragma once

class Gamepad{
public:
  Gamepad(int port, double stickDeadzone, double triggerDeadzone);
 
  double getTwist();
  frc::Rotation2d getLeftStickDirection();
  frc::Rotation2d getRightStickDirection();
  frc2::Trigger noBumpers();
  frc2::Trigger leftBumperOnly();
  frc2::Trigger rightBumperOnly();
  frc2::Trigger bothBumpers();
  frc2::Trigger noTriggers();
  frc2::Trigger leftTriggerOnly();
  frc2::Trigger rightTriggerOnly();
  frc2::Trigger bothTriggers();
  frc2::Trigger leftYTrigger();
  frc2::Trigger leftXTrigger();
  frc2::Trigger rightYTrigger();
  frc2::Trigger rightXTrigger();
  frc2::Trigger rightStick();
  frc2::Trigger leftStick();
  frc2::CommandPtr rumbleCommand(double intensity); // combinar
  frc2::Trigger a();
	frc2::Trigger b();
	frc2::Trigger x();
	frc2::Trigger y();
  frc2::Trigger start();
	frc2::Trigger select();
  frc2::Trigger upDpad();
	frc2::Trigger downDpad();
	frc2::Trigger leftDpad();
	frc2::Trigger rightDpad();
  
  frc2::CommandXboxController* Gamepad::getController();

  
private:
  frc::Rotation2d storedLeftStickDirection;
  frc::Rotation2d storedRightStickDirection;
  double stickDeadzone;
  double triggerDeadzone;
  
  frc2::CommandXboxController* m_controller;
  frc2::CommandGenericHID* m_genericHID;
};
