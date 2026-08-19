// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.robots;

import com.overture.lib.simulation.SimCANCoderManager;
import com.overture.lib.simulation.SimDutyCycleEncoderManager;
import com.overture.lib.simulation.SimMotorManager;
import com.overture.lib.simulation.SimPhotonVisionManager;
import com.overture.lib.simulation.SimPigeonManager;
import com.overture.lib.utils.Logging;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;

/**
 * Implementation of TimedRobot that allows to seamlessly change between simulation and a real
 * robot.
 *
 * <p>The C++ version selects the simulation bridge at compile time with {@code __FRC_ROBORIO__};
 * Java has no preprocessor, so the managers are always present and only driven when {@link
 * RobotBase#isSimulation()} is true.
 */
public class OverRobot extends TimedRobot {
  /** Bridges the TalonFXs to an external physics simulation. */
  public final SimMotorManager simMotorManager = SimMotorManager.getInstance();

  /** Bridges the Pigeon to an external physics simulation. */
  public final SimPigeonManager simPigeonManager = SimPigeonManager.getInstance();

  /** Bridges the CANcoders to an external physics simulation. */
  public final SimCANCoderManager simCANCoderManager = SimCANCoderManager.getInstance();

  /** Bridges the duty cycle encoders to an external physics simulation. */
  public final SimDutyCycleEncoderManager simDutyCycleEncoderManager =
      SimDutyCycleEncoderManager.getInstance();

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
      NetworkTableInstance.getDefault().stopServer();
      NetworkTableInstance.getDefault().startClient4("Offseason 2024");
      NetworkTableInstance.getDefault().setServer("127.0.0.1");

      addPeriodic(
          () -> {
            simPigeonManager.update();
            simCANCoderManager.update();
            simDutyCycleEncoderManager.update();
            simMotorManager.update();
            simPhotonVisionManager.update();
          },
          0.005);
    }

    Logging.startLogging();
  }
}
