// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.subsystems.leds;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertThrows;

import com.overture.lib.subsystems.leds.LedsManager.LedStripRange;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import java.util.Map;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Covers the slicing and validation LedsManager does around the LED buffer.
 *
 * <p>Only {@code new AddressableLED(port)} needs the HAL; the buffer and its views are plain Java,
 * so the interesting behaviour is testable on a desktop with no strip attached. The manager is
 * shared across the class because an AddressableLED permanently claims its PWM channel for the life
 * of the JVM, and the validation cases never reach that line.
 */
class LedsManagerTest {
  private static final int kStripLength = 24;

  private static LedsManager leds;

  @BeforeAll
  static void setUp() {
    leds = SharedTestLeds.get();
  }

  @Test
  void rangeEndingBeforeItStartsIsRejected() {
    assertThrows(
        IllegalArgumentException.class,
        () -> new LedsManager(1, kStripLength, Map.of("bad", new LedStripRange(5, 2))));
  }

  @Test
  void zeroLengthRangeIsRejected() {
    // endLed <= startLed, so a single LED slice is rejected too. Worth pinning: it is the C++
    // behaviour, and it means a one LED indicator has to be written as a two LED range.
    assertThrows(
        IllegalArgumentException.class,
        () -> new LedsManager(1, kStripLength, Map.of("bad", new LedStripRange(3, 3))));
  }

  @Test
  void rangeRunningPastTheEndOfTheStripIsRejected() {
    assertThrows(
        IllegalArgumentException.class,
        () -> new LedsManager(1, kStripLength, Map.of("bad", new LedStripRange(8, kStripLength))));
  }

  @Test
  void unknownStripNameThrows() {
    assertThrows(IllegalArgumentException.class, () -> leds.getLedStrip("nosuchstrip"));
  }

  @Test
  void viewCoversBothEndsOfTheRange() {
    AddressableLEDBufferView front = leds.getLedStrip(SharedTestLeds.kFront);
    assertNotNull(front);
    // 0..3 inclusive is four LEDs, not three.
    assertEquals(4, front.getLength());
  }

  @Test
  void reversedViewWritesInTheOppositeOrder() {
    AddressableLEDBufferView forward = leds.getLedStrip(SharedTestLeds.kBack);
    AddressableLEDBufferView reversed = leds.getLedStrip(SharedTestLeds.kBackReversed);

    // Both views cover physical LEDs 4..7. Writing through the reversed one and reading through
    // the forward one is what proves the reversal actually reaches the buffer, without needing
    // LedsManager to expose the buffer itself.
    for (int i = 0; i < reversed.getLength(); i++) {
      reversed.setRGB(i, i + 1, 0, 0);
    }

    assertEquals(4, forward.getLED8Bit(0).red);
    assertEquals(3, forward.getLED8Bit(1).red);
    assertEquals(2, forward.getLED8Bit(2).red);
    assertEquals(1, forward.getLED8Bit(3).red);
  }
}
