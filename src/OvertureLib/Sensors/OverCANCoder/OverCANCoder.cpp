// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "OverCANCoder.h"

OverCANCoder::OverCANCoder(int _id, double offset, std::string _bus) {
    canCoder = new CANcoder(_id, _bus);
    canCoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue::Signed_PlusMinusHalf;
    canCoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue::CounterClockwise_Positive;
    canCoderConfiguration.MagnetSensor.MagnetOffset = offset / 360;
    canCoder->GetConfigurator().Apply(canCoderConfiguration);
}

double OverCANCoder::getAbsolutePosition() {
    canCoder->GetAbsolutePosition().SetUpdateFrequency(200_Hz);
    return canCoder->GetAbsolutePosition().Refresh().GetValue().value();
}