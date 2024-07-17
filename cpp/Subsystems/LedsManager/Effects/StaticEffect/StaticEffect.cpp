// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "StaticEffect.h"

StaticEffect::StaticEffect(LedsManager* leds, LedStripName name, frc::Color8Bit color, bool addRequirement) {
  ledStrip = leds->getLedStrip(name);
  this->color = color;
  if(addRequirement){
    AddRequirements({leds});
  }
}

// Called when the command is initially scheduled.
void StaticEffect::Initialize() {
  std::for_each(ledStrip.begin(), ledStrip.end(), [&](frc::AddressableLED::LEDData& ledData) {
      ledData.SetRGB(color.red, color.green, color.blue);
  });
}

// Called repeatedly when this Command is scheduled to run
void StaticEffect::Execute() {}

// Called once the command ends or is interrupted.
void StaticEffect::End(bool interrupted) {}

// Returns true when the command should end.
bool StaticEffect::IsFinished() {
  return false;
}
