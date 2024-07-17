// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SimCANCoderManager.h"
#include <iostream>
#include <frc/RobotController.h>
#include <frc/MathUtil.h>

SimCANCoderManager* SimCANCoderManager ::instancePtr = NULL; 

SimCANCoderManager::SimCANCoderManager(){

}

void SimCANCoderManager::AddSimCANCoderCandidate(OverCANCoder* canCoder){
    canCodersToRegister.emplace_back(canCoder);
}

void SimCANCoderManager::RegisterSimCANCoder(OverCANCoder* canCoder){
    if(canCoder == NULL || canCoder == 0){
        throw std::invalid_argument("SimCANCoderManager given null pointer!");
    }


    if(!this->CANIDToCANCoderNameMap.contains(canCoder->GetDeviceID())){
        std::cout << "SimCANCoderManager Warning: Tried to register a CANCoder for simulation(" << canCoder->GetDeviceID() << ") that is not in the given map" << std::endl;
        return;
    }

    std::string canCoderName = this->CANIDToCANCoderNameMap[canCoder->GetDeviceID()];


    if(this->registeredCANCoders.contains(canCoderName)){
        std::cout << "SimMotorManager Warning: Tried to register a motor for simulation that was already registered" << std::endl;
        return;
    }

    std::shared_ptr<nt::NetworkTable> ntable = ntInst.GetTable(canCoderName);

    CANCoderNTPair newCANCoderPair;
    newCANCoderPair.canCoder = canCoder;
    newCANCoderPair.ntable = ntable;

    this->registeredCANCoders[canCoderName] = std::move(newCANCoderPair);
}

void SimCANCoderManager::Init(const std::map<unsigned int, NTCANCoderName> CANIDToCANCoderNameMap){
    this->robotName = robotName;
    this->CANIDToCANCoderNameMap = CANIDToCANCoderNameMap;

    std::for_each(canCodersToRegister.begin(), canCodersToRegister.end(), std::bind(&SimCANCoderManager::RegisterSimCANCoder, this, std::placeholders::_1));
}

void SimCANCoderManager::Update(){
    for (auto canCoderIterator = this->registeredCANCoders.begin(); canCoderIterator != this->registeredCANCoders.end(); canCoderIterator++) {
        CANCoderNTPair pair = canCoderIterator->second;

        std::shared_ptr<nt::NetworkTable> ntable = pair.ntable;
        OverCANCoder* canCoder = pair.canCoder;

        ctre::phoenix6::sim::CANcoderSimState& simState = canCoder->GetSimState();
        simState.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

        units::turn_t position = units::turn_t(ntable->GetEntry("cancoder_position").GetDouble(0));
        const auto absoluteSensorRangeConfig = canCoder->getConfiguration().MagnetSensor.AbsoluteSensorRange;

        if(absoluteSensorRangeConfig == ctre::phoenix6::signals::AbsoluteSensorRangeValue::Signed_PlusMinusHalf){
            position = frc::InputModulus(position, units::turn_t(-0.5), units::turn_t(0.5));
        }else if(absoluteSensorRangeConfig == ctre::phoenix6::signals::AbsoluteSensorRangeValue::Unsigned_0To1){
            position = frc::InputModulus(position, units::turn_t(0.0), units::turn_t(1.0));
        }

        simState.SetRawPosition(position);

        units::turns_per_second_t speed = units::turns_per_second_t(ntable->GetEntry("cancoder_speed").GetDouble(0));
        simState.SetVelocity(speed);
    }

    ntInst.Flush();
}

