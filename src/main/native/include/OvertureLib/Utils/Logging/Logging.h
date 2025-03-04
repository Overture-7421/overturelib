// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/phoenix6/SignalLogger.hpp>
#include <frc/geometry/Pose2d.h>

class Logging {
public:
	static void LogPose2d(std::string_view path, frc::Pose2d pose,
			units::second_t latencySeconds = 0_s) {
		ctre::phoenix6::SignalLogger::WriteDoubleArray(path,
				std::array<double, 3> { pose.X().value(), pose.Y().value(),
						pose.Rotation().Degrees().value() }, "",
				latencySeconds);
	}

	static void LogBoolean(std::string_view path, bool value) {
		ctre::phoenix6::SignalLogger::WriteBoolean(path, value);
	}

	static void LogAngle(std::string_view path, units::degree_t angle) {
		ctre::phoenix6::SignalLogger::WriteDouble(path, angle.value(),
				"Degrees");
	}

	static void LogDistance(std::string_view path, units::meter_t distance) {
		ctre::phoenix6::SignalLogger::WriteDouble(path, distance.value(),
				"Meters");
	}

	static void StartLogging() {
		ctre::phoenix6::SignalLogger::Start();
	}

	static void StopLogging() {
		ctre::phoenix6::SignalLogger::Stop();
	}
};
