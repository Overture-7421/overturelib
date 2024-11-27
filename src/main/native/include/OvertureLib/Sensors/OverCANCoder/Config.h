#pragma once

#include "OvertureLib/Sensors/OverCANCoder/OverCANCoder.h"

struct CanCoderConfig {
	int CanCoderId = -1;
	SensorDirectionValue SensorDirection =
			SensorDirectionValue::CounterClockwise_Positive;
	units::turn_t Offset = 0_deg;
};
