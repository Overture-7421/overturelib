// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/phoenix6/SignalLogger.hpp>
#include <frc/geometry/Pose2d.h>

class Logging: public ctre::phoenix6::SignalLogger {
public:
	static void LogPose2d(std::string_view path, frc::Pose2d pose,
			units::second_t latencySeconds = 0_s) {
		ctre::phoenix6::SignalLogger::WriteDoubleArray(path,
				std::array<double, 3> { pose.X().value(), pose.Y().value(),
						pose.Rotation().Degrees().value() }, "",
				latencySeconds);
	}

	static void StartLogging() {
		ctre::phoenix6::SignalLogger::Start();
	}

	static void StopLogging() {
		ctre::phoenix6::SignalLogger::Stop();
	}
};
