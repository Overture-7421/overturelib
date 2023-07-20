// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <AHRS.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/XboxController.h>
#include <frc2/command/button/Trigger.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SubsystemBase.h>

#include "OvertureLib/subsystems/Swerve/SwerveModule/SwerveModule.h"

class SwerveChassis: public frc2::SubsystemBase {
public:
  SwerveChassis();
  void setModulePositions(std::array<frc::Translation2d, 4>* positions);
  void setModulesRatios(double turnRatio, double driveRatio, double wheelDiameter);
  void setModules(SwerveModule* frontLeft, SwerveModule* frontRight, SwerveModule* backleft, SwerveModule* backRight);
  void setRotatorPID(double kP, double kI, double kD);
  void setDrivePID(double kP, double kI, double kD);
  void setFeedForward(units::volt_t kS, units::volt_t kV, units::volt_t kA);
  void setUseRawVoltageSpeed(bool set);

  void setTargetAngle(double targetAngle);
  void setSpeed(frc::ChassisSpeeds speeds);
  void setWheelVoltage(double voltage);

  frc::Pose2d getOdometry();
  void resetOdometry(frc::Pose2d initPose);
  double getHeadingRate();
  const frc::SwerveDriveKinematics<4>& getKinematics();
  void addVisionMeasurement(frc::Pose2d pose, units::second_t Latency);
  void resetNavx(double angle = 0);

  void setModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates);
  wpi::array<frc::SwerveModuleState, 4> getModuleStates();
  wpi::array<frc::SwerveModulePosition, 4> getModulePosition();

  double getPitch();
  double getYaw();
  double getRoll();

  void updateOdometry();
  void shuffleboardPeriodic();

protected:
  double linearX;
  double linearY;
  double angular;

  frc::XboxController controller{ 0 };
  frc2::Trigger resetNavxButton{ [this] {return controller.GetBackButton();} };

private:
  AHRS navx{ frc::SPI::Port::kMXP };

  SwerveModule* frontLeftModule;
  SwerveModule* frontRightModule;
  SwerveModule* backLeftModule;
  SwerveModule* backRightModule;

  double wheelVoltage;
  double targetAngle;

  std::array<frc::Translation2d, 4>* modulePos;

  frc::SwerveDriveKinematics<4>* kinematics;

  std::array<frc::SwerveModulePosition, 4>* odometryPos;

  frc::SwerveDrivePoseEstimator<4>* odometry;


};