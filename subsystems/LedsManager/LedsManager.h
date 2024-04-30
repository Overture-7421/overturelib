// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/AddressableLED.h>
#include <networktables/NetworkTableInstance.h>
#include <map>

typedef std::string LedStripName;
typedef std::vector<frc::AddressableLED::LEDData> LedStrip;

class LedsManager : public frc2::SubsystemBase {
 public:
  struct LedStripRange {
    int startLed;
    int endLed;
    bool reversed = false;
  };

  LedsManager(int pwmPort, int ledLength, const std::map<LedStripName, LedStripRange>& ledStripMap);

  std::span<frc::AddressableLED::LEDData> getLedStrip(LedStripName name);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() final;
  
 private:
  frc::AddressableLED ledStrip;
  LedStrip ledBuffer;
  std::span<frc::AddressableLED::LEDData> ledBufferSpan;
  std::map<LedStripName, LedStripRange> ledStripMap;
};