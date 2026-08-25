// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.simulation.mechanisms;

import static edu.wpi.first.units.Units.Rotations;

import com.overture.lib.sensors.OverCANCoder;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;

/**
 * An absolute sensor that a simulated mechanism drives with its true position.
 *
 * <p>Mechanisms take one of these instead of a particular sensor type, so an arm measured by a
 * CANcoder and one measured by a through-bore encoder on a DIO port are the same code with a
 * different factory call. Both {@code of} overloads deal with the quirks of their sensor, which is
 * the part worth not rediscovering per season.
 */
@FunctionalInterface
public interface SimEncoder {
  /**
   * Reports where the mechanism is.
   *
   * @param position the mechanism's position
   * @param velocity how fast it is moving
   */
  void update(Angle position, AngularVelocity velocity);

  /**
   * Drives a CANcoder.
   *
   * @param canCoder the encoder to drive
   * @return a sim encoder writing to it
   */
  static SimEncoder of(OverCANCoder canCoder) {
    // Phoenix subtracts this from whatever setRawPosition is given, cancelling the magnet offset
    // the device applies on the way out. Without it a calibrated encoder reads its true angle plus
    // the calibration, and only in simulation, which invites zeroing the offsets for simulation and
    // calling it fixed.
    canCoder.getSimState().SensorOffset = canCoder.getConfiguration().MagnetSensor.MagnetOffset;

    return (position, velocity) -> {
      var sim = canCoder.getSimState();
      sim.setRawPosition(position);
      sim.setVelocity(velocity);
    };
  }

  /**
   * Drives a duty cycle encoder on a DIO port.
   *
   * <p>Two things differ from a CANcoder. WPILib's simulated duty cycle encoder has no speed
   * signal, so the velocity is dropped. And in simulation {@code get()} returns exactly what is
   * written here, skipping the range and zero mapping a real encoder applies, so this writes
   * rotations, which is what a real encoder built with WPILib's default full range reports. Build
   * one with a range of its own and simulation will disagree with the robot, and there is no getter
   * to check it against.
   *
   * @param encoder the encoder to drive
   * @return a sim encoder writing to it
   */
  static SimEncoder of(DutyCycleEncoder encoder) {
    DutyCycleEncoderSim sim = new DutyCycleEncoderSim(encoder);
    sim.setConnected(true);

    return (position, velocity) -> sim.set(position.in(Rotations));
  }
}
