package com.overture.lib.sensors;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;

/**
 * OverCANCoderConfig is a configuration class for OverCANCoder that allows for more configuration
 * options.
 */
public class OverCANCoderConfig {

  /** The CanCoder id */
  public int CanCoderId;

  /** The sensor direction */
  public SensorDirectionValue SensorDirection;

  /** The offset */
  public Angle Offset;

  /** Constructor for OverCANCoderConfig */
  public OverCANCoderConfig() {
    CanCoderId = -1;
    SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    Offset = Rotations.zero();
  }

  /**
   * Sets the CanCoder id
   *
   * @param CanCoderId The CanCoder id
   * @return The OverCANCoderConfig object
   */
  public OverCANCoderConfig withCanCoderId(int CanCoderId) {
    this.CanCoderId = CanCoderId;
    return this;
  }

  /**
   * Sets the sensor direction
   *
   * @param SensorDirection The sensor direction
   * @return The OverCANCoderConfig object
   */
  public OverCANCoderConfig withSensorDirection(SensorDirectionValue SensorDirection) {
    this.SensorDirection = SensorDirection;
    return this;
  }

  /**
   * Sets the offset
   *
   * @param Offset The offset
   * @return The OverCANCoderConfig object
   */
  public OverCANCoderConfig withOffset(Angle Offset) {
    this.Offset = Offset;
    return this;
  }
}
