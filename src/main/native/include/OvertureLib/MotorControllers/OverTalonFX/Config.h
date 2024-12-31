#pragma once

#include "OvertureLib/MotorControllers/ControllerNeutralMode/ControllerNeutralMode.h"
#include <ctre/phoenix6/configs/Configs.hpp>
#include <units/current.h>
#include <units/time.h>

struct OverTalonFXConfig {
	int MotorId = -1;

	ControllerNeutralMode NeutralMode = ControllerNeutralMode::Brake;
	bool Inverted = false;
	bool useFOC = false;

	ctre::phoenix6::configs::Slot0Configs PIDConfigs =
			ctre::phoenix6::configs::Slot0Configs();

	units::ampere_t CurrentLimit = 0_A;
	units::ampere_t StatorCurrentLimit = 0_A;
	units::ampere_t TriggerThreshold = 0_A;
	units::second_t TriggerThresholdTime = 0_s;
	units::second_t ClosedLoopRampRate = 0_s;
	units::second_t OpenLoopRampRate = 0_s;
};
