// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.simulation;

import com.ctre.phoenix6.sim.CANcoderSimState;
import com.overture.lib.sensors.OverCANCoder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/** Bridges every {@link OverCANCoder} to an external physics simulation over NetworkTables. */
public final class SimCANCoderManager {
  private static final SimCANCoderManager instance = new SimCANCoderManager();

  private final NetworkTableInstance ntInst = NetworkTableInstance.getDefault();

  private Map<Integer, String> canIdToCANCoderNameMap = new HashMap<>();
  private final Map<String, CANCoderNTPair> registeredCANCoders = new HashMap<>();
  private final List<OverCANCoder> canCodersToRegister = new ArrayList<>();
  private boolean initialized = false;

  private static final class CANCoderNTPair {
    private NetworkTable ntable;
    private OverCANCoder canCoder;
  }

  private SimCANCoderManager() {}

  /**
   * Returns the singleton instance.
   *
   * @return the sim CANcoder manager
   */
  public static SimCANCoderManager getInstance() {
    return instance;
  }

  /**
   * Queues a CANcoder to be registered once {@link #init(Map)} runs.
   *
   * @param canCoder the CANcoder to register
   */
  public void addSimCANCoderCandidate(OverCANCoder canCoder) {
    System.out.println("Adding sim cancoder candidate with id: " + canCoder.getDeviceID() + "...");
    canCodersToRegister.add(canCoder);
  }

  /**
   * Registers every queued CANcoder whose CAN id appears in the given map.
   *
   * @param canIdToCANCoderNameMap maps CAN id to the NetworkTables name of the simulated CANcoder
   */
  public void init(Map<Integer, String> canIdToCANCoderNameMap) {
    System.out.println("Initializing SimCANCoderManager...");
    this.canIdToCANCoderNameMap = canIdToCANCoderNameMap;
    System.out.println("Got " + this.canIdToCANCoderNameMap.size() + " mapped CANCoders...");

    System.out.println("Got " + canCodersToRegister.size() + " CANCoders to register...");

    canCodersToRegister.forEach(this::registerSimCANCoder);
    initialized = true;
  }

  private void registerSimCANCoder(OverCANCoder canCoder) {
    if (canCoder == null) {
      throw new IllegalArgumentException("SimCANCoderManager given null pointer!");
    }

    if (!canIdToCANCoderNameMap.containsKey(canCoder.getDeviceID())) {
      System.out.println(
          "SimCANCoderManager Warning: Tried to register a CANCoder for simulation("
              + canCoder.getDeviceID()
              + ") that is not in the given map");
      return;
    }

    String canCoderName = canIdToCANCoderNameMap.get(canCoder.getDeviceID());

    if (registeredCANCoders.containsKey(canCoderName)) {
      System.out.println(
          "SimCANCoderManager Warning: Tried to register a motor for simulation that was already"
              + " registered");
      return;
    }

    CANCoderNTPair newCANCoderPair = new CANCoderNTPair();
    newCANCoderPair.canCoder = canCoder;
    newCANCoderPair.ntable = ntInst.getTable(canCoderName);

    registeredCANCoders.put(canCoderName, newCANCoderPair);
  }

  /** Pushes the simulated sensor state into every CANcoder. Call this periodically. */
  public void update() {
    if (!initialized) {
      return;
    }

    for (CANCoderNTPair pair : registeredCANCoders.values()) {
      NetworkTable ntable = pair.ntable;
      OverCANCoder canCoder = pair.canCoder;

      CANcoderSimState simState = canCoder.getSimState();
      simState.setSupplyVoltage(RobotController.getBatteryVoltage());

      simState.setRawPosition(ntable.getEntry("cancoder_position").getDouble(0));
      simState.setVelocity(ntable.getEntry("cancoder_speed").getDouble(0));
    }

    ntInst.flush();
  }
}
