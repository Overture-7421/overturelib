// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.sensors;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.overture.lib.simulation.SimCANCoderManager;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotBase;

/** A CANcoder preconfigured from a {@link CanCoderConfig}. */
public class OverCANCoder extends CANcoder {
  private final CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();

  private final Alert isConnectedAlert =
      new Alert("Devices", "CANCoder is not connected", Alert.AlertType.kWarning);

  /**
   * Constructs an OverCANCoder and applies the given configuration.
   *
   * @param config the configuration of the CANcoder
   * @param bus the CAN bus the CANcoder lives on
   */
  public OverCANCoder(CanCoderConfig config, CANBus bus) {
    super(config.CanCoderId, bus);

    canCoderConfiguration.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(
        config.absoluteDiscontinuityPoint);
    canCoderConfiguration.MagnetSensor.withMagnetOffset(config.Offset);
    canCoderConfiguration.MagnetSensor.withSensorDirection(config.SensorDirection);

    getConfigurator().apply(canCoderConfiguration);

    if (RobotBase.isSimulation()) {
      SimCANCoderManager.getInstance().addSimCANCoderCandidate(this);
    }

    isConnectedAlert.setText("CANCoder " + config.CanCoderId + " is not connected");
  }

  /**
   * Returns the applied CANcoder configuration.
   *
   * @return the CANcoder configuration
   */
  public CANcoderConfiguration getConfiguration() {
    return canCoderConfiguration;
  }

  /** Raises the disconnection alert when the device stops responding. */
  public void updateConnectionAlert() {
    isConnectedAlert.set(!isConnected());
  }
}
