// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/CommandPS5Controller.h>
#include <frc2/command/button/CommandGenericHID.h>
#include <frc2/command/button/Trigger.h>
#include <math/Utils.h>

#pragma once

class OvertureController {
 public:
  OvertureController(int port, bool isXbox);

 private:
  bool isXbox;
  std::unique_ptr<frc2::CommandXboxController> xboxController;
  std::unique_ptr<frc2::CommandPS5Controller> ps5Controller;
  std::unique_ptr<frc2::CommandGenericHID> abstractedController;
  std::unique_ptr<frc2::CommandXboxController> emulatedController;

 public:
  frc2::Trigger a();
  frc2::Trigger b();
  frc2::Trigger x();
  frc2::Trigger y();
  frc2::Trigger leftBumper();
  frc2::Trigger rightBumper();
  frc2::Trigger leftTrigger(double threshold);
  frc2::Trigger rightTrigger(double threshold);
  frc2::Trigger leftStick();
  frc2::Trigger rightStick();
  frc2::Trigger start();
  frc2::Trigger select();
  frc2::Trigger trackpad();
  frc2::Trigger upDpad();
  frc2::Trigger downDpad();
  frc2::Trigger leftDpad();
  frc2::Trigger rightDpad();
  double getRightTriggerAxis();
  double getLeftTriggerAxis();
  double getLeftX();
  double getLeftY();
  double getRightX();
  double getRightY();
  frc2::CommandGenericHID getHID();
  frc2::CommandGenericHID getRumbleHID();
  void rumbleController(double leftIntensity, double rightIntensity);

};
