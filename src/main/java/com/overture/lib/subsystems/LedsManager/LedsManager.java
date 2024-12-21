package com.overture.lib.subsystems.LedsManager;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Map;

/** The LedsManager class is a wrapper for the LEDs on the robot. */
public class LedsManager extends SubsystemBase {
  private AddressableLED ledStrip;
  private AddressableLEDBuffer ledBuffer;
  private Map<String, LedStripRange> ledStripMap;

  /**
   * Creates a new LedsManager.
   *
   * @param pwmPort The PWM port of the LEDs.
   * @param ledLength The length of the LED strip.
   * @param ledStripMap The map of LED strip ranges.
   */
  public LedsManager(int pwmPort, int ledLength, Map<String, LedStripRange> ledStripMap) {
    for (LedStripRange ledStripRange : ledStripMap.values()) {
      ledStripRange.start = Math.max(0, Math.min(ledLength, ledStripRange.start));
      ledStripRange.end = Math.max(0, Math.min(ledLength, ledStripRange.end));
    }

    this.ledStrip = new AddressableLED(pwmPort);
    this.ledBuffer = new AddressableLEDBuffer(ledLength);
    this.ledStrip.setLength(ledLength);
    this.ledStrip.setData(ledBuffer);
    this.ledStrip.start();
    this.ledStripMap = ledStripMap;
  }

  /**
   * Gets the LED strip with the given name.
   *
   * @param name The name of the LED strip.
   * @return The LED strip.
   */
  public AddressableLEDBufferView getLedStrip(String name) {
    LedStripRange range = ledStripMap.get(name);
    if (range != null) {
      return new AddressableLEDBufferView(ledBuffer, range.start, range.end);
    }
    return null;
  }

  /** Periodically updates the LED strip. */
  @Override
  public void periodic() {
    ledStrip.setData(ledBuffer);
  }
}
