// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.HashMap;
import java.util.Map;

/** Owns one addressable LED strip and hands out named views into it. */
public class LedsManager extends SubsystemBase {
  /** A named, contiguous slice of the physical strip. */
  public static class LedStripRange {
    /** Index of the first LED in the slice, inclusive. */
    public int startLed;

    /** Index of the last LED in the slice, inclusive. */
    public int endLed;

    /** Whether the slice runs backwards along the physical strip. */
    public boolean reversed = false;

    /**
     * Constructs a range.
     *
     * @param startLed index of the first LED, inclusive
     * @param endLed index of the last LED, inclusive
     */
    public LedStripRange(int startLed, int endLed) {
      this.startLed = startLed;
      this.endLed = endLed;
    }

    /**
     * Constructs a range.
     *
     * @param startLed index of the first LED, inclusive
     * @param endLed index of the last LED, inclusive
     * @param reversed whether the slice runs backwards along the physical strip
     */
    public LedStripRange(int startLed, int endLed, boolean reversed) {
      this(startLed, endLed);
      this.reversed = reversed;
    }
  }

  private final AddressableLED ledStrip;
  private final AddressableLEDBuffer ledBuffer;
  private final Map<String, AddressableLEDBufferView> ledStripViews = new HashMap<>();

  /**
   * Constructs a LedsManager.
   *
   * @param pwmPort the PWM port the strip is wired to
   * @param ledLength how many LEDs are on the strip
   * @param ledStripMap named slices of the strip
   * @throws IllegalArgumentException if a slice is inverted or runs past the end of the strip
   */
  public LedsManager(int pwmPort, int ledLength, Map<String, LedStripRange> ledStripMap) {
    for (Map.Entry<String, LedStripRange> entry : ledStripMap.entrySet()) {
      LedStripRange range = entry.getValue();
      if (range.endLed <= range.startLed) {
        throw new IllegalArgumentException(
            "Led strip has an end led that is before the start led!!!");
      }

      if (range.startLed >= ledLength || range.endLed >= ledLength) {
        throw new IllegalArgumentException(
            "Led strip has a start or end led that is greater than the total length!!!");
      }
    }

    ledBuffer = new AddressableLEDBuffer(ledLength);

    for (Map.Entry<String, LedStripRange> entry : ledStripMap.entrySet()) {
      LedStripRange range = entry.getValue();
      AddressableLEDBufferView view = ledBuffer.createView(range.startLed, range.endLed);
      ledStripViews.put(entry.getKey(), range.reversed ? view.reversed() : view);
    }

    ledStrip = new AddressableLED(pwmPort);
    ledStrip.setLength(ledLength);
    ledStrip.setData(ledBuffer);
    ledStrip.start();
  }

  /**
   * Returns the named slice of the strip.
   *
   * @param name the name the slice was registered under
   * @return a writable view of that slice
   * @throws IllegalArgumentException if no slice was registered under that name
   */
  public AddressableLEDBufferView getLedStrip(String name) {
    AddressableLEDBufferView view = ledStripViews.get(name);
    if (view == null) {
      throw new IllegalArgumentException("No led strip registered under the name " + name);
    }
    return view;
  }

  @Override
  public void periodic() {
    ledStrip.setData(ledBuffer);
  }
}
