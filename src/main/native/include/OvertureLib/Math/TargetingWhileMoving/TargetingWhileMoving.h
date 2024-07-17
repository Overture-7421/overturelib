// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>

#include <units/acceleration.h>
#include <units/velocity.h>
#include <map>

#include "OvertureLib/Math/ChassisAccels.h"
#include "OvertureLib/Math/InterpolatingTable/InterpolatingTable.h"

typedef InterpolatingTable<units::meter_t, units::second_t> DistanceToTravelTimeTable;

class TargetingWhileMoving {
public:
	TargetingWhileMoving(DistanceToTravelTimeTable distanceToTravelTime, units::second_t accelCompFactor = 0.01_s);

	void setTargetLocation(frc::Translation2d targetLocation);

	frc::Translation2d getMovingTarget(const frc::Pose2d& robotPose, const frc::ChassisSpeeds& fieldRelativeSpeed, const ChassisAccels& fieldRelativeAccel);
private:
	frc::Translation2d targetLocation;
	DistanceToTravelTimeTable distanceToTravelTime;
	units::second_t accelCompFactor;
};
