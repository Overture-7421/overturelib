// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "OvertureController.h"

OvertureController::OvertureController(int port, bool isXbox){
    this->isXbox = isXbox;
    if(isXbox = true){
        // xboxController = std::make_unique<new frc::XboxController{port}>;
    }
        
};

frc2::Trigger OvertureController::a(){
        return (isXbox = true) ? xboxController->GetAButton : ps5Controller->GetCrossButton;
        };