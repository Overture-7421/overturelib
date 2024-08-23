#pragma once

#include "OvertureLib/Sensors/OverCANCoder/OverCANCoder.h"

struct CanCoderConfig {
	int CanCoderId = -1;
	bool EncoderInverted = false;
	units::turn_t Offset = 0_deg;
};
