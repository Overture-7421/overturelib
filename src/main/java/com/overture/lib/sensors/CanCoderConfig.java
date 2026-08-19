// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.sensors;

import com.ctre.phoenix6.signals.SensorDirectionValue;

/** Configuration bundle for {@link OverCANCoder}. */
public class CanCoderConfig {
  /** CAN id of the CANcoder. */
  public int CanCoderId = -1;

  /** Which rotation direction the sensor reports as positive. */
  public SensorDirectionValue SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

  /** Magnet offset, in rotations. */
  public double Offset = 0.0;

  /** Where the absolute position wraps around, in rotations. */
  public double absoluteDiscontinuityPoint = 0.5;
}
