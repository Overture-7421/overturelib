// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandPtr.h>
#include <pathplanner/lib/commands/PPSwerveControllerCommand.h>
#include <pathplanner/lib/PathPlannerTrajectory.h>
#include <pathplanner/lib/PathPlanner.h>

#include "OvertureLib/Subsystems/Swerve/SwerveChassis/SwerveChassis.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */

class SwerveTrajectories
  : public frc2::CommandHelper<frc2::CommandBase, SwerveTrajectories> {
public:
  SwerveTrajectories(SwerveChassis* swerveChassis, pathplanner::PathPlannerTrajectory trajectory, frc2::PIDController xController, frc2::PIDController yController, frc2::PIDController rController);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

private:
  /* Subsystems */
  SwerveChassis* swerveChassis;

  pathplanner::PPSwerveControllerCommand* alignCommand;
  pathplanner::PathPlannerTrajectory m_trajectory;

  frc2::PIDController m_xController;
  frc2::PIDController m_yController;
  frc2::PIDController m_rController;
};
