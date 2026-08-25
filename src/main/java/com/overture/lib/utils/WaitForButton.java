// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.utils;

import com.overture.lib.gamepads.OverXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;

/** Commands that block until a raw button is pressed. */
public final class WaitForButton {
  private WaitForButton() {}

  /**
   * Returns a command that waits until a button on an Xbox controller is pressed.
   *
   * @param gamepad the controller to watch
   * @param buttonNumber the raw button number
   * @return the command
   */
  public static Command waitForButton(OverXboxController gamepad, int buttonNumber) {
    return Commands.waitUntil(() -> gamepad.getHID().getRawButton(buttonNumber));
  }

  /**
   * Returns a command that waits until a button on a generic HID is pressed.
   *
   * @param gamepad the HID to watch
   * @param buttonNumber the raw button number
   * @return the command
   */
  public static Command waitForButton(CommandGenericHID gamepad, int buttonNumber) {
    return Commands.waitUntil(() -> gamepad.getHID().getRawButton(buttonNumber));
  }
}
