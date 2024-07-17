// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <frc/TimedRobot.h>
#include "RobotConstants.h"

/**
 * Implementation of TimedRobot that allows to seamlessly change between simulation (using Gazebo) and a real robot.
*/

#ifdef __FRC_ROBORIO__
class OverRobot : public frc::TimedRobot {
public:
    OverRobot(units::second_t period = 20_ms) : frc::TimedRobot(period) {

    }
};

#else

#include <frc/IterativeRobotBase.h>
#include <hal/Types.h>
#include <units/math.h>
#include <units/time.h>
#include <wpi/priority_queue.h>
#include <networktables/NetworkTableEntry.h>
#include "OvertureLib/Simulation/SimMotorManager/SimMotorManager.h"
#include "OvertureLib/Simulation/SimPigeonManager/SimPigeonManager.h"
#include "OvertureLib/Simulation/SimCANCoderManager/SimCANCoderManager.h"
#include "OvertureLib/Simulation/SimDutyCycleEncoderManager/SimDutyCycleEncoderManager.h"


using namespace frc;

class OverRobot : public frc::IterativeRobotBase {
public:
    OverRobot(units::second_t period = 20_ms);
    void StartCompetition() override;
    void EndCompetition() override;
    units::second_t GetSimulationTime();
    void AddPeriodic(std::function<void()> callback, units::second_t period,
                   units::second_t offset = 0_s);
    ~OverRobot() override;

    SimMotorManager* simMotorManager = SimMotorManager::GetInstance();
    SimPigeonManager* simPigeonManager = SimPigeonManager::GetInstance();
	SimCANCoderManager* simCANCoderManager = SimCANCoderManager::GetInstance();
	SimDutyCycleEncoderManager* simDutyCycleEncoderManager = SimDutyCycleEncoderManager::GetInstance();

private:
    class Callback {
    public:
        std::function<void()> func;
        units::second_t period;
        units::second_t expirationTime;

        /**
         * Construct a callback container.
         *
         * @param func      The callback to run.
         * @param startTime The common starting point for all callback scheduling.
         * @param period    The period at which to run the callback.
         * @param offset    The offset from the common starting time.
         */
        Callback(std::function<void()> func, units::second_t startTime,
                units::second_t period, units::second_t offset, OverRobot* robot)
        : func{std::move(func)},
            period{period},
            expirationTime{startTime + offset +
                            units::math::floor(
                                (robot->GetSimulationTime() - startTime) / period) *
                                period +
                            period} {}

        bool operator>(const Callback& rhs) const {
            return expirationTime > rhs.expirationTime;
        }
    };

    units::second_t m_startTime, m_lastTime;
    nt::NetworkTableEntry simTimeEntry;
    wpi::priority_queue<Callback, std::vector<Callback>, std::greater<Callback>> m_callbacks;
};

#endif