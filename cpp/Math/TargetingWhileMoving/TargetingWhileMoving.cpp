// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "TargetingWhileMoving.h"

TargetingWhileMoving::TargetingWhileMoving(DistanceToTravelTimeTable distanceToTravelTime, units::second_t accelCompFactor) 
: distanceToTravelTime(distanceToTravelTime), accelCompFactor(accelCompFactor){

};

void TargetingWhileMoving::setTargetLocation(frc::Translation2d targetLocation) {
    this->targetLocation = targetLocation;
}

frc::Translation2d TargetingWhileMoving::getMovingTarget(const frc::Pose2d& robotPose, const frc::ChassisSpeeds& fieldRelativeSpeed, const ChassisAccels& fieldRelativeAccel) {
    frc::Translation2d robotLocation = robotPose.Translation();
    units::meter_t distanceToTarget = targetLocation.Distance(robotLocation);
    units::second_t shotTime = distanceToTravelTime[distanceToTarget];

    frc::Translation2d movingGoalLocation, testGoalLocation;
    
    for(int i = 0; i < 5; i++) {
        units::meter_t virtualGoalX = targetLocation.X() - shotTime * (fieldRelativeSpeed.vx + fieldRelativeAccel.ax * accelCompFactor);
        units::meter_t virtualGoalY = targetLocation.Y() - shotTime * (fieldRelativeSpeed.vy + fieldRelativeAccel.ay * accelCompFactor);
        testGoalLocation = {virtualGoalX, virtualGoalY};

        units::meter_t distanceToTestGoal = testGoalLocation.Distance(robotLocation);
        units::second_t newShotTime = distanceToTravelTime[distanceToTestGoal];

        if(units::math::abs(newShotTime-shotTime) <= 0.010_s){
            i=4;
        }

        if(i == 4){
            movingGoalLocation = testGoalLocation;
        }else{
            shotTime = newShotTime;
        }
    }

    return movingGoalLocation;
}