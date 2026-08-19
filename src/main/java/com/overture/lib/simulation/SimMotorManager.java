// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.simulation;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.overture.lib.motorcontrollers.OverTalonFX;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/** Bridges every {@link OverTalonFX} to an external physics simulation over NetworkTables. */
public final class SimMotorManager {
  private static final SimMotorManager instance = new SimMotorManager();

  private final NetworkTableInstance ntInst = NetworkTableInstance.getDefault();

  private Map<Integer, String> canIdToMotorNameMap = new HashMap<>();
  private final Map<String, MotorNTPair> registeredMotors = new HashMap<>();
  private final List<OverTalonFX> motorsToRegister = new ArrayList<>();
  private boolean initialized = false;

  private static final class MotorNTPair {
    private NetworkTable ntable;
    private OverTalonFX motor;
  }

  private SimMotorManager() {
    System.out.println("Created new Sim Motor Manager...");
  }

  /**
   * Returns the singleton instance.
   *
   * @return the sim motor manager
   */
  public static SimMotorManager getInstance() {
    return instance;
  }

  /**
   * Queues a motor to be registered once {@link #init(Map)} runs.
   *
   * @param motor the motor to register
   */
  public void addSimMotorCandidate(OverTalonFX motor) {
    System.out.println("Adding sim motor candidate with id: " + motor.getDeviceID() + "...");
    motorsToRegister.add(motor);
  }

  /**
   * Registers every queued motor whose CAN id appears in the given map.
   *
   * @param canIdToMotorNameMap maps CAN id to the NetworkTables name of the simulated motor
   */
  public void init(Map<Integer, String> canIdToMotorNameMap) {
    System.out.println("Initializing SimMotorManager...");
    this.canIdToMotorNameMap = canIdToMotorNameMap;
    System.out.println("Got " + this.canIdToMotorNameMap.size() + " mapped motors...");

    System.out.println("Got " + motorsToRegister.size() + " motors to register...");
    for (OverTalonFX motor : motorsToRegister) {
      registerSimMotor(motor);
    }

    initialized = true;
  }

  private void registerSimMotor(OverTalonFX motor) {
    if (motor == null) {
      throw new IllegalArgumentException("SimMotorManager given null pointer!");
    }

    if (!canIdToMotorNameMap.containsKey(motor.getDeviceID())) {
      System.out.println(
          "SimMotorManager Warning: Tried to register a motor for simulation("
              + motor.getDeviceID()
              + ") that is not in the given map");
      return;
    }

    String motorName = canIdToMotorNameMap.get(motor.getDeviceID());

    if (registeredMotors.containsKey(motorName)) {
      System.out.println(
          "SimMotorManager Warning: Tried to register a motor for simulation that was already"
              + " registered");
      return;
    }

    MotorNTPair newMotorPair = new MotorNTPair();
    newMotorPair.motor = motor;
    newMotorPair.ntable = ntInst.getTable(motorName);

    registeredMotors.put(motorName, newMotorPair);
  }

  /** Exchanges motor state with the external simulation. Call this periodically. */
  public void update() {
    if (!initialized) {
      return;
    }

    for (MotorNTPair pair : registeredMotors.values()) {
      NetworkTable ntable = pair.ntable;
      OverTalonFX motor = pair.motor;

      TalonFXSimState simState = motor.getSimState();

      simState.setSupplyVoltage(RobotController.getBatteryVoltage());

      boolean inverted =
          motor.getCTREConfig().MotorOutput.Inverted == InvertedValue.Clockwise_Positive;

      ntable.getEntry("software_inverted").setBoolean(inverted);

      ntable.getEntry("voltage_applied").setDouble(simState.getMotorVoltage());

      simState.setRawRotorPosition(ntable.getEntry("encoder_position").getDouble(0));
      simState.setRotorVelocity(ntable.getEntry("encoder_speed").getDouble(0));
    }

    ntInst.flush();
  }
}
