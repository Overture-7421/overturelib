// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>
#include <ctre/phoenix6/signals/SpnEnums.hpp>

using namespace ctre::phoenix6::hardware;
using namespace ctre::phoenix6::configs;
using namespace ctre::phoenix6::signals;

class OverCANCoder {
public:
  OverCANCoder(int _id, double offset, std::string _bus);
  double getAbsolutePosition();
  CANcoder* getCanCoder() { return canCoder; }

private:
  CANcoder* canCoder;
  CANcoderConfiguration canCoderConfiguration;
};
