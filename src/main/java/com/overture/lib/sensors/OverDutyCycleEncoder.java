package com.overture.lib.sensors;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

/**
 * OverDutyCycleEncoder is a wrapper class for the DutyCycleEncoder that allows for more
 * configuration options.
 */
public class OverDutyCycleEncoder extends DutyCycleEncoder {
  /**
   * Constructor for OverDutyCycleEncoder.
   *
   * @param channel The channel that the encoder is connected to.
   */
  public OverDutyCycleEncoder(int channel) {
    super(channel);
  }
}
