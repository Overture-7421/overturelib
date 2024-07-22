#pragma once

#include <frc/kinematics/ChassisSpeeds.h>

class SpeedsHelper {
public:
	virtual void AlterSpeed(frc::ChassisSpeeds &inputSpeed) = 0;
};
