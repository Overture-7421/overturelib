// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.robots;

import com.overture.lib.simulation.SimPhotonVisionManager;
import com.overture.lib.utils.Logging;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;

/**
 * Implementation of TimedRobot that allows to seamlessly change between simulation and a real
 * robot.
 *
 * <p>The physics used to live in a second program, so this class turned the robot into a
 * NetworkTables client of localhost and pumped every device's state across at 200 Hz. The physics
 * now runs in this process: the drivetrain is stepped by SwerveBase and each mechanism by the
 * subsystem that owns it, so the only thing still driven from here is PhotonVision's simulated
 * cameras, which belong to no subsystem. The robot is the NetworkTables server again, the way
 * WPILib expects, so a dashboard can connect to it directly.
 */
public class OverRobot extends TimedRobot {
  /** Drives PhotonVision's simulated cameras. */
  public final SimPhotonVisionManager simPhotonVisionManager = SimPhotonVisionManager.getInstance();

  /** Constructs an OverRobot with the default 20 ms period. */
  public OverRobot() {
    this(0.02);
  }

  /**
   * Constructs an OverRobot.
   *
   * @param period the robot loop period, in seconds
   */
  public OverRobot(double period) {
    super(period);

    if (RobotBase.isSimulation()) {
      addPeriodic(simPhotonVisionManager::update, 0.005);
    }

    Logging.startLogging();
  }
}
