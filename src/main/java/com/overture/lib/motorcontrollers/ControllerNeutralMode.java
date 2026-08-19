// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.motorcontrollers;

/** The neutral mode determines how the motor controller behaves when not receiving any signal. */
public enum ControllerNeutralMode {
  /** The motor spins freely when neutral. */
  Coast,
  /** The motor resists motion when neutral. */
  Brake
}
