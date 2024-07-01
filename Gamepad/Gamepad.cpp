// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Gamepad.h"

Gamepad::Gamepad(int port, double deadzone){
    this->deadzone = deadzone;
    m_controller = new frc2::CommandXboxController(port);
    m_genericHID = new frc2::CommandGenericHID(port);
};

frc2::CommandXboxController* Gamepad::getController(){
    return m_controller;
}

double Gamepad::getTwist(){
    double right = m_controller->GetRightTriggerAxis();
    double left = m_controller->GetLeftTriggerAxis();
    double value = right - left;
    return value;
};


frc::Rotation2d Gamepad::getLeftStickDirection(){
    double x = Utils::ApplyAxisFilter(-m_controller->GetLeftX(), deadzone);
    double y = Utils::ApplyAxisFilter(-m_controller->GetLeftY(), deadzone);

    return frc::Rotation2d(x, y);
};

frc::Rotation2d Gamepad::getRightStickDirection(){
    double x = Utils::ApplyAxisFilter(-m_controller->GetRightX(), deadzone);
    double y = Utils::ApplyAxisFilter(-m_controller->GetRightY(), deadzone);

    return frc::Rotation2d(x, y);
};

frc2::Trigger Gamepad::noBumpers(){
    return !m_controller->RightBumper() && !m_controller->LeftBumper();
}   

frc2::Trigger Gamepad::leftBumperOnly(){
    return m_controller->LeftBumper() && !m_controller->RightBumper();
}

frc2::Trigger Gamepad::rightBumperOnly(){
    return m_controller->RightBumper() && !m_controller->LeftBumper();
}

frc2::Trigger Gamepad::bothBumpers(){
    return m_controller->RightBumper() && m_controller->LeftBumper();
}

frc2::Trigger Gamepad::noTriggers(){
    return !m_controller->LeftTrigger(0) && !m_controller->RightTrigger(0);
}

frc2::Trigger Gamepad::leftTriggerOnly(){
    return m_controller->LeftTrigger(0) && !m_controller->RightTrigger(0);
}

frc2::Trigger Gamepad::rightTriggerOnly(){
    return m_controller->RightTrigger(0) && !m_controller->LeftTrigger(0);
}

frc2::Trigger Gamepad::bothTriggers(){
    return m_controller->LeftTrigger(0) && m_controller->RightTrigger(0);
}

frc2::Trigger Gamepad::a(){
    return m_controller->A();
};

frc2::Trigger Gamepad::b(){
    return m_controller->B();
}

frc2::Trigger Gamepad::x(){
    return m_controller->X();
}

frc2::Trigger Gamepad::y(){
    return m_controller->Y();
}

frc2::Trigger Gamepad::start(){
    return m_controller->Start();
}

frc2::Trigger Gamepad::select(){
    return m_controller->Back();
}

frc2::Trigger Gamepad::upDpad(){
    return m_genericHID->POVUp();
}

frc2::Trigger Gamepad::downDpad(){
    return m_genericHID->POVDown();
}

frc2::Trigger Gamepad::leftDpad(){
    return m_genericHID->POVLeft();
}

frc2::Trigger Gamepad::rightDpad(){
    return m_genericHID->POVRight();
}

frc2::CommandPtr Gamepad::rumbleCommand(double intensity){
    m_controller->SetRumble(frc::GenericHID::RumbleType::kBothRumble, intensity);
}

