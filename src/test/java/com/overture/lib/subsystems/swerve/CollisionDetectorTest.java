// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.subsystems.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/** Covers the collision detector. Pure arithmetic, so the clock is passed in rather than read. */
class CollisionDetectorTest {
  private static final double kThresholdGs = 1.5;
  private static final double kHoldSeconds = 0.5;

  private CollisionDetector detector() {
    return new CollisionDetector(kThresholdGs, kHoldSeconds);
  }

  @Test
  void startsQuiet() {
    assertFalse(detector().isColliding(0.0), "nothing has happened yet");
  }

  @Test
  void hardDrivingIsNotACollision() {
    CollisionDetector detector = detector();

    // About what a swerve pulls flat out. If this tripped, the flag would be up for most of a
    // match.
    assertFalse(detector.update(1.0, 0.0));
    assertFalse(detector.update(1.2, 0.02));
  }

  @Test
  void aSpikeTrips() {
    CollisionDetector detector = detector();

    assertTrue(detector.update(1.8, 0.0));
    assertEquals(1.8, detector.getLastAccelerationGs(), 1e-9);
  }

  @Test
  void theFlagIsHeldAfterTheImpulsePasses() {
    CollisionDetector detector = detector();
    detector.update(1.8, 0.0);

    // The hit is over by the very next loop. Without the hold, nothing downstream would ever see
    // it.
    assertTrue(detector.update(0.1, 0.02), "still held one loop later");
    assertTrue(detector.update(0.1, 0.4), "still held before the window closes");
    assertFalse(detector.update(0.1, 0.6), "released once the window has passed");
  }

  @Test
  void aSecondHitRestartsTheHold() {
    CollisionDetector detector = detector();
    detector.update(1.8, 0.0);
    detector.update(1.9, 0.4);

    assertTrue(detector.update(0.1, 0.6), "the second hit pushed the window out");
    assertFalse(detector.update(0.1, 0.95));
  }

  @Test
  void aSaturatedPigeonStillTrips() {
    // A Pigeon 2 clips at just under 2 g, so a genuinely violent hit arrives as the rail value
    // rather than as something larger. The threshold sits below it precisely so that still counts.
    assertTrue(detector().update(1.99993896484375, 0.0));
  }
}
