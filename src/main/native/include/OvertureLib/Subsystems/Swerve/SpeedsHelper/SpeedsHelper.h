#pragma once

#include <frc/kinematics/ChassisSpeeds.h>

class SpeedsHelper {
public:
	virtual void alterSpeed(frc::ChassisSpeeds &inputSpeed) = 0;
};
