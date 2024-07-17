// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "OvertureLib/Subsystems/LedsManager/Effects/BlinkEffect/BlinkEffect.h"
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>

BlinkEffect::BlinkEffect(LedsManager* leds, LedStripName name, frc::Color8Bit color, units::second_t period, bool addRequirement) : color(color) {
	ledStrip = leds->getLedStrip(name);
	this->period = period;

	if (addRequirement) {
		AddRequirements({ leds });
	}
}

// Called when the command is initially scheduled.
void BlinkEffect::Initialize() {
	startTime = frc::Timer::GetFPGATimestamp();
}

// Called repeatedly when this Command is scheduled to run
void BlinkEffect::Execute() {
	auto elapsedTime = frc::Timer::GetFPGATimestamp() - startTime;
	double blinkMultiplier = std::cos(elapsedTime.value() * 2.0 * M_PI * 1.0 / period.value()) * 0.5 + 0.5;

	double red = color.red * blinkMultiplier;
	double green = color.green * blinkMultiplier;
	double blue = color.blue * blinkMultiplier;

	std::for_each(ledStrip.begin(), ledStrip.end(), [&](frc::AddressableLED::LEDData& ledData) {
		ledData.SetRGB(red, green, blue);
	});
}

// Called once the command ends or is interrupted.
void BlinkEffect::End(bool interrupted) {}

// Returns true when the command should end.
bool BlinkEffect::IsFinished() {
	return false;
}
