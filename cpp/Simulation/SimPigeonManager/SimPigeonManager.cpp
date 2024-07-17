// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SimPigeonManager.h"
#include <networktables/NetworkTable.h>
#include <frc/RobotController.h>
#include <iostream>

SimPigeonManager* SimPigeonManager ::instancePtr = NULL; 


SimPigeonManager::SimPigeonManager(){

}

void SimPigeonManager::SetSimPigeon(OverPigeon* pigeon){
    if(pigeon == NULL || pigeon == 0){
        throw std::invalid_argument("SimPigeonManager given null pointer!");
    }

    this->pigeon = pigeon;
}

void SimPigeonManager::Init(std::string imuName){

    if(pigeon == NULL) {
        std::cout << "SimPigeonManager Warning: No Pigeon created" << std::endl;
        return;
    }

    std::shared_ptr<nt::NetworkTable> ntable = ntInst.GetTable(imuName);
    rollEntry = ntable->GetEntry("roll");
    pitchEntry = ntable->GetEntry("pitch");
    yawEntry = ntable->GetEntry("yaw");
    
    pigeonSimState = &pigeon->GetSimState();
    std::cout << "SimPigeonManager Info: Initialized for Pigeon with ID: " << pigeon->GetDeviceID() << std::endl;
}

void SimPigeonManager::Update(){
    if(pigeon == NULL || pigeonSimState == NULL) {
        return;
    }
    
    pigeonSimState->SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

    pigeonSimState->SetRoll(units::angle::degree_t(rollEntry.GetDouble(0)));
    pigeonSimState->SetPitch(units::angle::degree_t(pitchEntry.GetDouble(0)));
    pigeonSimState->SetRawYaw(units::angle::degree_t(yawEntry.GetDouble(0)));
    
    ntInst.Flush();
}


