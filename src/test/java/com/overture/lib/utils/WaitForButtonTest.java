// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.utils;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.overture.lib.gamepads.OverXboxController;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/** Covers both WaitForButton overloads against a simulated driver station. */
class WaitForButtonTest {
  private static final int kPort = 0;
  private static final int kButton = 3;

  @BeforeAll
  static void setUp() {
    HAL.initialize(500, 0);
    DriverStationSim.setJoystickButtonCount(kPort, 8);
  }

  @AfterEach
  void releaseButton() {
    setButton(false);
  }

  private static void setButton(boolean pressed) {
    DriverStationSim.setJoystickButton(kPort, kButton, pressed);
    DriverStationSim.notifyNewData();
  }

  @Test
  void genericHidOverloadWaitsForThePress() {
    CommandGenericHID hid = new CommandGenericHID(kPort);
    Command command = WaitForButton.waitForButton(hid, kButton);

    setButton(false);
    command.initialize();
    command.execute();
    assertFalse(command.isFinished(), "should still be waiting while the button is up");

    setButton(true);
    command.execute();
    assertTrue(command.isFinished(), "should finish once the button goes down");
  }

  @Test
  void xboxOverloadWaitsForThePress() {
    OverXboxController gamepad = new OverXboxController(kPort, 0.2, 0.2);
    Command command = WaitForButton.waitForButton(gamepad, kButton);

    setButton(false);
    command.initialize();
    command.execute();
    assertFalse(command.isFinished(), "should still be waiting while the button is up");

    setButton(true);
    command.execute();
    assertTrue(command.isFinished(), "should finish once the button goes down");
  }

  @Test
  void aButtonAlreadyHeldFinishesImmediately() {
    // The C++ version behaves the same way. Worth pinning, because it is the difference between
    // "wait for a press" and "wait for a rising edge" and callers rely on the former.
    setButton(true);

    Command command = WaitForButton.waitForButton(new CommandGenericHID(kPort), kButton);
    command.initialize();
    command.execute();

    assertTrue(command.isFinished(), "a button already held should not block");
  }
}
