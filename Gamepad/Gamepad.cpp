// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Gamepad.h"

Gamepad::Gamepad(int port, double stickDeadzone, double triggerDeadzone){
    this->stickDeadzone = stickDeadzone;
    this->triggerDeadzone = triggerDeadzone;
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
    double x = Utils::ApplyAxisFilter(-m_controller->GetLeftX(), stickDeadzone);
    double y = Utils::ApplyAxisFilter(-m_controller->GetLeftY(), stickDeadzone);

    return frc::Rotation2d(x, y);
};

frc::Rotation2d Gamepad::getRightStickDirection(){
    double x = Utils::ApplyAxisFilter(-m_controller->GetRightX(), stickDeadzone);
    double y = Utils::ApplyAxisFilter(-m_controller->GetRightY(), stickDeadzone);

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
    return !m_controller->LeftTrigger(triggerDeadzone) && !m_controller->RightTrigger(triggerDeadzone);
}

frc2::Trigger Gamepad::leftTriggerOnly(){
    return m_controller->LeftTrigger(triggerDeadzone) && !m_controller->RightTrigger(triggerDeadzone);
}

frc2::Trigger Gamepad::rightTriggerOnly(){
    return m_controller->RightTrigger(triggerDeadzone) && !m_controller->LeftTrigger(triggerDeadzone);
}

frc2::Trigger Gamepad::bothTriggers(){
    return m_controller->LeftTrigger(triggerDeadzone) && m_controller->RightTrigger(triggerDeadzone);
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

frc2::Trigger Gamepad::leftYTrigger(){
    {[this]{return std::abs(m_controller->GetLeftY()) >= stickDeadzone;};
}
}

frc2::Trigger Gamepad::leftXTrigger(){
    {[this]{return std::abs(m_controller->GetLeftX()) >= stickDeadzone;};
}
}

frc2::Trigger Gamepad::rightYTrigger(){
    {[this]{return std::abs(m_controller->GetRightY()) >= stickDeadzone;}; 
}
}

frc2::Trigger Gamepad::rightXTrigger(){
    {[this]{return std::abs(m_controller->GetRightX()) >= stickDeadzone;};
}
}

frc2::Trigger Gamepad::rightStick(){
    {[this]{return std::abs(m_controller->GetRightX()) >= stickDeadzone || m_controller->GetRightX() >= stickDeadzone;};};
    
}

frc2::Trigger Gamepad::leftStick(){
    {[this]{return std::abs(m_controller->GetLeftX()) >= stickDeadzone || m_controller->GetLeftX() >= stickDeadzone;};};
}