// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "OvertureController.h"

OvertureController::OvertureController(int port, bool isXbox, int emulatedPort) {
	this->isXbox = isXbox;
	if (isXbox) {
		xboxController = std::make_shared<frc2::CommandXboxController>(port);
		abstractedController = std::make_shared<frc2::CommandGenericHID>(xboxController);
	} else {
		ps5Controller = std::make_shared<frc2::CommandPS5Controller>(port);
		emulatedController = std::make_shared<frc2::CommandXboxController>(emulatedPort);
		abstractedController = std::make_shared<frc2::CommandGenericHID>(ps5Controller);
	}

};

frc2::Trigger OvertureController::a(){
        return (isXbox == true) ? xboxController->A() : ps5Controller->Cross();
};

frc2::Trigger OvertureController::b(){
        return (isXbox == true) ? xboxController->B() : ps5Controller->Circle();
};

frc2::Trigger OvertureController::x(){
        return (isXbox == true) ? xboxController->X() : ps5Controller->Square();
};

frc2::Trigger OvertureController::y(){
        return (isXbox == true) ? xboxController->Y() : ps5Controller->Triangle();
};

frc2::Trigger OvertureController::leftBumper(){
        return (isXbox == true) ? xboxController->LeftBumper() : ps5Controller->L1();
};

frc2::Trigger OvertureController::rightBumper(){
        return (isXbox == true) ? xboxController->RightBumper() : ps5Controller->R1();
};

/*
Necesitan new trigger


frc2::Trigger OvertureController::leftTrigger(double threshold){
        return (isXbox == true) ? xboxController->LeftTrigger(threshold) :

frc::Trigger OvertureController::leftTrigger(double threshold){
        return (isXbox == true) ? xboxController->LeftTrigger(threshold) :
}
}*/

frc2::Trigger OvertureController::leftStick(){
        return (isXbox == true) ? xboxController->LeftStick() : ps5Controller->L3();
};

frc2::Trigger OvertureController::rightStick(){
        return (isXbox == true) ? xboxController->RightStick() : ps5Controller->R3();
}

frc2::Trigger OvertureController::start(){
        return (isXbox == true) ? xboxController->Start() : ps5Controller->Options();
};

/*
ps5Controller Create() necesita parámetros

frc2::Trigger OvertureController::select(){
        return (isXbox == true) ? xboxController->Back() : ps5Controller->Create());
};
*/

/*
Necesita un new Trigger
frc2::Trigger OvertureController::trackpad() {
        return (isXbox == true) ? new... : ps5Controller.Touchpad();

*/

frc2::Trigger OvertureController::upDpad() {
        return abstractedController->POVUp();
    }

 frc2::Trigger OvertureController::downDpad() {
        return abstractedController->POVDown();
    }

frc2::Trigger OvertureController::leftDpad() {
        return abstractedController->POVLeft();
    }

frc2::Trigger OvertureController::rightDpad() {
        return abstractedController->POVRight();
    }

double OvertureController::getRightTriggerAxis() {
        return (isXbox == true) ? xboxController->GetRightTriggerAxis()
                : std::min(std::abs(ps5Controller->GetR2Axis()), 0.99);
    }

double OvertureController::getRightTriggerAxis() {
        return (isXbox == true) ? xboxController->GetRightTriggerAxis()
                : std::min(std::abs(ps5Controller->GetL2Axis()), 0.99);
    }

double OvertureController::getLeftX() {
        return (isXbox == true) ? xboxController->GetLeftX() : ps5Controller->GetLeftX();
    }

double OvertureController::getLeftY() {
        return (isXbox == true) ? xboxController->GetLeftY() : ps5Controller->GetLeftY();
    }

double OvertureController::getRightX() {
        return (isXbox == true) ? xboxController->GetRightX() : ps5Controller->GetRightX();
    }

double OvertureController::getRightY() {
        return (isXbox == true) ? xboxController->GetRightY() : ps5Controller->GetRightY();
    }

/* 
No hay method llamado GetHID

frc2::CommandGenericHID OvertureController::getHID() {
        return (isXbox == true) ? xboxController->() : ps5Controller->();
    } 

frc2::CommandGenericHID OvertureController::getHID() {
        return (isXbox == true) ? xboxController->() : ps5Controller->();
    }

*/

void OvertureController::rumbleController(double leftIntensity, double rightIntensity) {
        getRumbleHID().SetRumble(frc::GenericHID::RumbleType::kLeftRumble, leftIntensity);
        getRumbleHID().SetRumble(frc::GenericHID::RumbleType::kRightRumble, rightIntensity);
    }