// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.motorcontrollers;

import com.ctre.phoenix6.configs.Slot0Configs;

/**
 * Configuration bundle for {@link OverTalonFX}. Field names mirror the C++ OverTalonFXConfig struct
 * so configurations port across one to one.
 */
public class OverTalonFXConfig {
  /** CAN id of the motor. */
  public int MotorId = -1;

  /** Behaviour of the motor when it receives no signal. */
  public ControllerNeutralMode NeutralMode = ControllerNeutralMode.Brake;

  /** Whether the motor output is inverted. */
  public boolean Inverted = false;

  /** Whether to use Field Oriented Control on control requests. */
  public boolean useFOC = false;

  /** Slot 0 closed loop gains. */
  public Slot0Configs PIDConfigs = new Slot0Configs();

  /** Supply current lower limit, in amps. */
  public double CurrentLimit = 0.0;

  /** Stator current limit, in amps. */
  public double StatorCurrentLimit = 0.0;

  /** Supply current limit that triggers the lower limit, in amps. */
  public double TriggerThreshold = 0.0;

  /** How long the trigger threshold may be exceeded before limiting, in seconds. */
  public double TriggerThresholdTime = 0.0;

  /** Closed loop voltage ramp period, in seconds. */
  public double ClosedLoopRampRate = 0.0;

  /** Open loop voltage ramp period, in seconds. */
  public double OpenLoopRampRate = 0.0;
}
