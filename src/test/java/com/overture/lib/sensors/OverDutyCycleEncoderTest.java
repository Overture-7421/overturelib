// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.sensors;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.overture.lib.simulation.SimDutyCycleEncoderManager;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.Map;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;

/**
 * Covers the duty cycle encoder together with the simulation manager it registers itself into.
 *
 * <p>Neither class had ever been executed: the simulation harness initialised the manager with an
 * empty map, so no encoder was ever constructed and the registration path never ran.
 */
@TestMethodOrder(org.junit.jupiter.api.MethodOrderer.MethodName.class)
class OverDutyCycleEncoderTest {
  private static final int kChannel = 4;
  private static final String kSimName = "SimEncoders/testEncoder";

  private static OverDutyCycleEncoder encoder;

  @BeforeAll
  static void setUp() {
    HAL.initialize(500, 0);
    encoder = new OverDutyCycleEncoder(kChannel);
  }

  @Test
  void aRemembersTheChannelItWasBuiltOn() {
    assertEquals(kChannel, encoder.getSourceChannel());
  }

  @Test
  void bSimulatedPositionReachesTheEncoderThroughNetworkTables() {
    // The constructor queued this encoder as a candidate; init binds it to the NT name, and
    // update pumps the value across. This is the whole simulation path in one go.
    SimDutyCycleEncoderManager manager = SimDutyCycleEncoderManager.getInstance();
    manager.init(Map.of(kChannel, kSimName));

    NetworkTableInstance.getDefault()
        .getTable(kSimName)
        .getEntry("cancoder_position")
        .setDouble(0.25);

    manager.update();

    assertEquals(0.25, encoder.get(), 1e-9);
    assertTrue(encoder.isConnected(), "an encoder with a published position should read connected");
  }

  @Test
  void cUpdateTracksLaterValues() {
    NetworkTableInstance.getDefault()
        .getTable(kSimName)
        .getEntry("cancoder_position")
        .setDouble(0.75);

    SimDutyCycleEncoderManager.getInstance().update();

    assertEquals(0.75, encoder.get(), 1e-9);
  }
}
