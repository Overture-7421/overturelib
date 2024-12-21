package com.overture.lib.subsystems.LedsManager;

/** The number of LEDs on the robot. */
public class LedStripRange {
  /** The start of the range. */
  public int start;
  /** The end of the range. */
  public int end;
  /** Whether the range is reversed. */
  public boolean isReversed;

  /**
   * Creates a new LedStripRange.
   *
   * @param start The start of the range.
   * @param end The end of the range.
   * @param isReversed Whether the range is reversed.
   */
  public LedStripRange(int start, int end, boolean isReversed) {
    this.start = start;
    this.end = end;
    this.isReversed = false;
  }
}
