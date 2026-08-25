// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.sensors;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.wpilibj.Alert;

/** A Pigeon 2 that raises a driver station alert when it stops responding. */
public class OverPigeon extends Pigeon2 {
  private final Alert isConnectedAlert =
      new Alert("Devices", "Pigeon is not connected", Alert.AlertType.kWarning);

  /**
   * Constructs an OverPigeon on the default CAN bus.
   *
   * @param deviceId the CAN id of the Pigeon
   */
  public OverPigeon(int deviceId) {
    this(deviceId, new CANBus(""));
  }

  /**
   * Constructs an OverPigeon.
   *
   * @param deviceId the CAN id of the Pigeon
   * @param canbus the CAN bus the Pigeon lives on
   */
  public OverPigeon(int deviceId, CANBus canbus) {
    super(deviceId, canbus);

    isConnectedAlert.setText("Pigeon " + deviceId + " is not connected");
  }

  /** Raises the disconnection alert when the device stops responding. */
  public void updateConnectionAlert() {
    isConnectedAlert.set(!isConnected());
  }
}
