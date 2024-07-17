// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "Sensors/OverCANCoder/OverCANCoder.h"
#include <map>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

typedef std::string NTCANCoderName;

class SimCANCoderManager {
 public:
  void AddSimCANCoderCandidate(OverCANCoder* motor);
  void Init(const std::map<unsigned int, NTCANCoderName> CANIDToCANCoderNameMap);
  void Update();

  static SimCANCoderManager* GetInstance(){
    // If there is no instance of class
    // then we can create an instance.
    if (instancePtr == NULL) {
      // We can access private members 
      // within the class.
      instancePtr = new SimCANCoderManager(); 
        
      // returning the instance pointer
      return instancePtr; 
    } else {
      // if instancePtr != NULL that means 
      // the class already have an instance. 
      // So, we are returning that instance 
      // and not creating new one.
      return instancePtr;
    }
  }
  SimCANCoderManager(const SimCANCoderManager& obj) = delete; 

private:
  SimCANCoderManager();
  void RegisterSimCANCoder(OverCANCoder* canCoder);

  struct CANCoderNTPair {
    std::shared_ptr<nt::NetworkTable> ntable;
    OverCANCoder* canCoder;
  };

  nt::NetworkTableInstance ntInst = nt::NetworkTableInstance::GetDefault();

  std::map<unsigned int, NTCANCoderName> CANIDToCANCoderNameMap;
  std::map<NTCANCoderName, CANCoderNTPair> registeredCANCoders;
  std::vector<OverCANCoder*> canCodersToRegister;
  std::string robotName;


  static SimCANCoderManager* instancePtr; 
};
