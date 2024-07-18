// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "OvertureLib/Robots/OverRobot/OverRobot.h"

#ifndef __FRC_ROBORIO__

#include <hal/DriverStation.h>
#include <hal/FRCUsageReporting.h>
#include <hal/Notifier.h>

#include <networktables/NetworkTableInstance.h>
#include <iostream>
#include <frc/DriverStation.h>
#include <photon/PhotonCamera.h>

OverRobot::OverRobot(units::second_t period) : frc::IterativeRobotBase(period) {
	std::puts("\nSimulation OverRobot Started!!!!");

	const auto ntable = nt::NetworkTableInstance::GetDefault().GetTable("nt_simworld");
	simTimeEntry = ntable->GetEntry("sim_time");

	m_startTime = GetSimulationTime();
	AddPeriodic([=, this] { LoopFunc(); }, period);

	int32_t status = 0;
	FRC_CheckErrorStatus(status, "InitializeNotifier");

	HAL_Report(HALUsageReporting::kResourceType_Framework,
		HALUsageReporting::kFramework_Timed);

	frc::DriverStation::SilenceJoystickConnectionWarning(true);

	photon::PhotonCamera::SetVersionCheckEnabled(false);
}

void OverRobot::StartCompetition() {
	RobotInit();

	if constexpr (IsSimulation()) {
		SimulationInit();
	}

	// Tell the DS that the robot is ready to be enabled
	std::puts("\n********** Robot program startup complete **********");
	HAL_ObserveUserProgramStarting();

	// Loop forever, calling the appropriate mode-dependent function
	while (true) {
		// We don't have to check there's an element in the queue first because
		// there's always at least one (the constructor adds one). It's reenqueued
		// at the end of the loop.
		auto callback = m_callbacks.pop();

		units::second_t simTime = GetSimulationTime();

		while (simTime - m_lastTime < GetPeriod()) {
			simTime = GetSimulationTime();
			std::this_thread::sleep_for(std::chrono::microseconds(500));
		}

		double curTime = simTime.value();

		m_lastTime = simTime;

		simPigeonManager->Update();
		simCANCoderManager->Update();
		simDutyCycleEncoderManager->Update();
		simMotorManager->Update();
		callback.func();

		callback.expirationTime += callback.period;

		m_callbacks.push(std::move(callback));
		// Process all other callbacks that are ready to run
		while (static_cast<double>(m_callbacks.top().expirationTime * 1e6) <=
			curTime) {
			callback = m_callbacks.pop();

			callback.func();

			callback.expirationTime += callback.period;
			m_callbacks.push(std::move(callback));
		}
	}
}

void OverRobot::EndCompetition() {
	nt::NetworkTableInstance ntInst = nt::NetworkTableInstance::GetDefault();
	ntInst.Disconnect();
	nt::NetworkTableInstance::GetDefault().Destroy(ntInst);
}

OverRobot::~OverRobot() {

}

units::second_t OverRobot::GetSimulationTime() {
	return units::second_t(simTimeEntry.GetDouble(0));
}

void OverRobot::AddPeriodic(std::function<void()> callback,
	units::second_t period, units::second_t offset) {
	m_callbacks.emplace(callback, m_startTime, period, offset, this);
}

#endif