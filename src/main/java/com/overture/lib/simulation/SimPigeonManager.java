// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.simulation;

import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.overture.lib.sensors.OverPigeon;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;

/** Bridges an {@link OverPigeon} to an external physics simulation over NetworkTables. */
public final class SimPigeonManager {
  private static final SimPigeonManager instance = new SimPigeonManager();

  private OverPigeon pigeon = null;
  private Pigeon2SimState pigeonSimState = null;

  private final NetworkTableInstance ntInst = NetworkTableInstance.getDefault();
  private NetworkTableEntry rollEntry;
  private NetworkTableEntry pitchEntry;
  private NetworkTableEntry yawEntry;
  private boolean initialized = false;

  private SimPigeonManager() {}

  /**
   * Returns the singleton instance.
   *
   * @return the sim pigeon manager
   */
  public static SimPigeonManager getInstance() {
    return instance;
  }

  /**
   * Sets the Pigeon this manager drives.
   *
   * @param pigeon the Pigeon to simulate
   */
  public void setSimPigeon(OverPigeon pigeon) {
    if (pigeon == null) {
      throw new IllegalArgumentException("SimPigeonManager given null pointer!");
    }

    this.pigeon = pigeon;
  }

  /**
   * Binds the Pigeon to a NetworkTables entry group.
   *
   * @param imuName the NetworkTables name of the simulated IMU
   */
  public void init(String imuName) {
    if (pigeon == null) {
      System.out.println("SimPigeonManager Warning: No Pigeon created");
      return;
    }

    NetworkTable ntable = ntInst.getTable(imuName);
    rollEntry = ntable.getEntry("roll");
    pitchEntry = ntable.getEntry("pitch");
    yawEntry = ntable.getEntry("yaw");

    pigeonSimState = pigeon.getSimState();
    System.out.println(
        "SimPigeonManager Info: Initialized for Pigeon with ID: " + pigeon.getDeviceID());
    initialized = true;
  }

  /** Pushes the simulated attitude into the Pigeon. Call this periodically. */
  public void update() {
    if (!initialized) {
      return;
    }

    if (pigeon == null || pigeonSimState == null) {
      return;
    }

    pigeonSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    // Pigeon2SimState takes degrees for its double overloads, unlike TalonFXSimState and
    // CANcoderSimState which take rotations. The NetworkTables entries are already degrees.
    pigeonSimState.setRoll(rollEntry.getDouble(0));
    pigeonSimState.setPitch(pitchEntry.getDouble(0));
    pigeonSimState.setRawYaw(yawEntry.getDouble(0));

    ntInst.flush();
  }
}
