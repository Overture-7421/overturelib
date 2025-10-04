#pragma once

#include "OvertureLib/Sensors/OverCANCoder/OverCANCoder.h"

struct CanCoderConfig {
	int CanCoderId = -1;
	ctre::phoenix6::signals::SensorDirectionValue SensorDirection =
			ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive;
	units::turn_t Offset = 0_deg;
	units::turn_t absoluteDiscontinuityPoint = 0.5_tr;
};
