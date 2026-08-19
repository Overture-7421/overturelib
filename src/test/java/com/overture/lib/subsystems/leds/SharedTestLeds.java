// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.subsystems.leds;

import com.overture.lib.subsystems.leds.LedsManager.LedStripRange;
import edu.wpi.first.hal.HAL;
import java.util.Map;

/**
 * The one LedsManager the LED tests share.
 *
 * <p>The HAL allows a single AddressableLED per program, mirroring the roboRIO, which has one LED
 * output. A second one fails with "No available resources to allocate", so every LED test has to
 * work off the same strip. The slices below are disjoint so that tests writing into one cannot
 * disturb another regardless of the order they run in.
 */
public final class SharedTestLeds {
  /** Slice used by the view and reversal tests. */
  public static final String kFront = "front";

  /** Slice used by the reversal test, forward direction. */
  public static final String kBack = "back";

  /** The same LEDs as {@link #kBack}, reversed. */
  public static final String kBackReversed = "backReversed";

  /** Slice the effects are pointed at. */
  public static final String kTarget = "target";

  /** Slice next to the target, used to prove effects stay inside their own view. */
  public static final String kNeighbour = "neighbour";

  private static LedsManager instance;

  private SharedTestLeds() {}

  /**
   * Returns the shared manager, building it on first use.
   *
   * @return the shared LedsManager
   */
  public static synchronized LedsManager get() {
    if (instance == null) {
      HAL.initialize(500, 0);
      instance =
          new LedsManager(
              0,
              24,
              Map.of(
                  kFront, new LedStripRange(0, 3),
                  kBack, new LedStripRange(4, 7),
                  kBackReversed, new LedStripRange(4, 7, true),
                  kTarget, new LedStripRange(8, 11),
                  kNeighbour, new LedStripRange(12, 15)));
    }
    return instance;
  }
}
