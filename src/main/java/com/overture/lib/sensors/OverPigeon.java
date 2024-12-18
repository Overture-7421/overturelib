package com.overture.lib.sensors;

import com.ctre.phoenix6.hardware.Pigeon2;

/** OverPigeon is a wrapper class for Pigeon2 that allows for more configuration options. */
public class OverPigeon extends Pigeon2 {

  /**
   * Constructor for OverPigeon
   *
   * @param deviceNumber The device number
   * @param canbus The canbus
   */
  public OverPigeon(int deviceNumber, String canbus) {
    super(deviceNumber, canbus);
  }

  /**
   * Constructor for OverPigeon
   *
   * @param deviceNumber The device number
   */
  public OverPigeon(int deviceNumber) {
    super(deviceNumber);

	// TODO: Implement Simulation for OverPigeon
  }
}
