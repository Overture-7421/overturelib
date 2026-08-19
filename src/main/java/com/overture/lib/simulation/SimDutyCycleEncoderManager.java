// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.simulation;

import com.overture.lib.sensors.OverDutyCycleEncoder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Bridges every {@link OverDutyCycleEncoder} to an external physics simulation over NetworkTables.
 */
public final class SimDutyCycleEncoderManager {
  private static final SimDutyCycleEncoderManager instance = new SimDutyCycleEncoderManager();

  private final NetworkTableInstance ntInst = NetworkTableInstance.getDefault();

  private Map<Integer, String> chnToDutyCycleEncoderNameMap = new HashMap<>();
  private final Map<String, DutyCycleEncoderNTPair> registeredDutyCycleEncoders = new HashMap<>();
  private final List<OverDutyCycleEncoder> dutyCycleEncodersToRegister = new ArrayList<>();
  private boolean initialized = false;

  private static final class DutyCycleEncoderNTPair {
    private NetworkTable ntable;
    private DutyCycleEncoderSim dutyCycleEncoder;
  }

  private SimDutyCycleEncoderManager() {}

  /**
   * Returns the singleton instance.
   *
   * @return the sim duty cycle encoder manager
   */
  public static SimDutyCycleEncoderManager getInstance() {
    return instance;
  }

  /**
   * Queues an encoder to be registered once {@link #init(Map)} runs.
   *
   * @param encoder the encoder to register
   */
  public void addSimDutyCycleEncoderCandidate(OverDutyCycleEncoder encoder) {
    dutyCycleEncodersToRegister.add(encoder);
  }

  /**
   * Registers every queued encoder whose channel appears in the given map.
   *
   * @param chnToDutyCycleEncoderNameMap maps DIO channel to the NetworkTables name of the simulated
   *     encoder
   */
  public void init(Map<Integer, String> chnToDutyCycleEncoderNameMap) {
    this.chnToDutyCycleEncoderNameMap = chnToDutyCycleEncoderNameMap;

    dutyCycleEncodersToRegister.forEach(this::registerSimDutyCycleEncoder);
    initialized = true;
  }

  private void registerSimDutyCycleEncoder(OverDutyCycleEncoder encoder) {
    if (encoder == null) {
      throw new IllegalArgumentException("SimDutyCycleEncoderManager given null pointer!");
    }

    if (!chnToDutyCycleEncoderNameMap.containsKey(encoder.getSourceChannel())) {
      System.out.println(
          "SimDutyCycleEncoderManager Warning: Tried to register a DutyCycleEncoder for"
              + " simulation("
              + encoder.getSourceChannel()
              + ") that is not in the given map");
      return;
    }

    String encoderName = chnToDutyCycleEncoderNameMap.get(encoder.getSourceChannel());

    if (registeredDutyCycleEncoders.containsKey(encoderName)) {
      System.out.println(
          "SimDutyCycleEncoderManager Warning: Tried to register a DutyCycleEncoder for simulation"
              + " that was already registered");
      return;
    }

    DutyCycleEncoderNTPair newDutyCycleEncoderPair = new DutyCycleEncoderNTPair();
    newDutyCycleEncoderPair.dutyCycleEncoder = new DutyCycleEncoderSim(encoder);
    newDutyCycleEncoderPair.ntable = ntInst.getTable(encoderName);

    newDutyCycleEncoderPair.dutyCycleEncoder.setConnected(true);
    registeredDutyCycleEncoders.put(encoderName, newDutyCycleEncoderPair);
  }

  /** Pushes the simulated sensor state into every encoder. Call this periodically. */
  public void update() {
    if (!initialized) {
      return;
    }

    for (DutyCycleEncoderNTPair pair : registeredDutyCycleEncoders.values()) {
      NetworkTable ntable = pair.ntable;
      DutyCycleEncoderSim encoder = pair.dutyCycleEncoder;

      encoder.set(ntable.getEntry("cancoder_position").getDouble(0));
      encoder.setConnected(ntable.getEntry("cancoder_position").exists());
    }

    ntInst.flush();
  }
}
