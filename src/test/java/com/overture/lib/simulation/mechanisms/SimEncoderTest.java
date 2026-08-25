// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.simulation.mechanisms;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Covers driving a duty cycle encoder from a simulated mechanism.
 *
 * <p>This replaces a test of the manager that used to carry the value in from a second program over
 * NetworkTables. That program is retired, so the path under test is the one a mechanism simulation
 * actually calls.
 *
 * <p>Only the duty cycle factory is exercised. The CANcoder one would pull in Phoenix, whose native
 * libraries are deliberately absent from the test runtime.
 */
class SimEncoderTest {
  private static final int kChannel = 4;
  private static final int kSecondChannel = 5;

  private static DutyCycleEncoder encoder;
  private static SimEncoder simEncoder;

  @BeforeAll
  static void setUp() {
    HAL.initialize(500, 0);
    encoder = new DutyCycleEncoder(kChannel);
    simEncoder = SimEncoder.of(encoder);
  }

  @Test
  void mechanismPositionReachesTheEncoderInRotations() {
    // A mechanism hands over an angle; the encoder reports rotations, so the conversion is the
    // thing
    // being checked. 90 degrees is a quarter turn.
    simEncoder.update(Degrees.of(90.0), DegreesPerSecond.of(0.0));

    assertEquals(0.25, encoder.get(), 1e-9);
  }

  @Test
  void laterPositionsTrack() {
    simEncoder.update(Rotations.of(0.75), DegreesPerSecond.of(12.0));
    assertEquals(0.75, encoder.get(), 1e-9);

    // The velocity argument is dropped rather than rejected: WPILib's simulated duty cycle encoder
    // has no speed signal. Position must still be the last thing written.
    simEncoder.update(Rotations.of(0.1), DegreesPerSecond.of(-500.0));
    assertEquals(0.1, encoder.get(), 1e-9);
  }

  @Test
  void drivingAnEncoderMarksItConnected() {
    // WPILib defaults a simulated encoder to connected, so this starts from disconnected. Otherwise
    // the assertion passes whether or not SimEncoder does anything, which is what it did at first.
    DutyCycleEncoder disconnected = new DutyCycleEncoder(kSecondChannel);
    new DutyCycleEncoderSim(disconnected).setConnected(false);
    assertFalse(disconnected.isConnected(), "should start disconnected for this to mean anything");

    SimEncoder.of(disconnected);

    assertTrue(disconnected.isConnected(), "a driven encoder should read connected");
  }
}
