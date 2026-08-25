// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.gamepads;

import com.overture.lib.utils.Logging;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;

/** A button console with connection telemetry. */
public class OverConsole extends CommandGenericHID {
  private final Alert isConnectedAlert =
      new Alert("Controllers", "Console is not connected", Alert.AlertType.kWarning);

  /**
   * Constructs an OverConsole.
   *
   * @param port the driver station port the console is plugged into
   */
  public OverConsole(int port) {
    super(port);
  }

  /** Publishes the connection state of the console. */
  public void updateTelemetry() {
    isConnectedAlert.set(!isConnected());

    Logging.logBoolean(
        "/Controllers/ConsolePad-" + getHID().getPort() + "/IsConnected", isConnected());
  }
}
