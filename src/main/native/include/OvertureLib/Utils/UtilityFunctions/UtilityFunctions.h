// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "OvertureLib/Subsystems/Swerve/SwerveChassis/SwerveChassis.h"
#include <frc/DriverStation.h>

bool isRedAlliance();
units::length::meter_t getDistanceToChassis(SwerveChassis *chassis,
		frc::Pose2d targetPose);
