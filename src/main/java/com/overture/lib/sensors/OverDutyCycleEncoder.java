// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.sensors;

import com.overture.lib.simulation.SimDutyCycleEncoderManager;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * A DutyCycleEncoder that registers itself with the simulation manager when running in simulation.
 *
 * <p>Unlike the C++ DutyCycleEncoder, the Java one does not expose the channel it was built on, so
 * this class remembers it for the simulation manager to key on.
 */
public class OverDutyCycleEncoder extends DutyCycleEncoder {
  private final int sourceChannel;

  /**
   * Constructs an OverDutyCycleEncoder.
   *
   * @param channel the DIO channel the encoder is wired to
   */
  public OverDutyCycleEncoder(int channel) {
    super(channel);
    this.sourceChannel = channel;

    if (RobotBase.isSimulation()) {
      SimDutyCycleEncoderManager.getInstance().addSimDutyCycleEncoderCandidate(this);
    }
  }

  /**
   * Returns the DIO channel this encoder was constructed on.
   *
   * @return the source channel
   */
  public int getSourceChannel() {
    return sourceChannel;
  }
}
