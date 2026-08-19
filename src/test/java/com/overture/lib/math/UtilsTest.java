// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.math;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/** Checks the axis filter curve ported from Math/Utils.h. */
class UtilsTest {
  private static final double kEpsilon = 1e-9;

  @Test
  void sgnMatchesTheCppThreeWayComparison() {
    assertEquals(1, Utils.sgn(0.5));
    assertEquals(-1, Utils.sgn(-0.5));
    assertEquals(0, Utils.sgn(0.0));
  }

  @Test
  void insideTheDeadzoneIsExactlyZero() {
    assertEquals(0.0, Utils.applyAxisFilter(0.0), kEpsilon);
    assertEquals(0.0, Utils.applyAxisFilter(0.04), kEpsilon);
    assertEquals(0.0, Utils.applyAxisFilter(-0.04), kEpsilon);
  }

  @Test
  void fullDeflectionMapsToFullOutput() {
    // At |axis| == 1 both curve terms reach 1, so the output saturates at exactly 1.
    assertEquals(1.0, Utils.applyAxisFilter(1.0), kEpsilon);
    assertEquals(-1.0, Utils.applyAxisFilter(-1.0), kEpsilon);
  }

  @Test
  void isSignPreservingAndOdd() {
    for (double v : new double[] {0.1, 0.25, 0.5, 0.75, 0.99}) {
      double positive = Utils.applyAxisFilter(v);
      double negative = Utils.applyAxisFilter(-v);
      assertTrue(positive > 0, "expected positive output for " + v);
      assertEquals(-positive, negative, kEpsilon, "filter should be an odd function");
    }
  }

  @Test
  void isMonotonicOutsideTheDeadzone() {
    double previous = 0;
    for (double v = 0.06; v <= 1.0; v += 0.01) {
      double current = Utils.applyAxisFilter(v);
      assertTrue(current > previous, "filter should increase at " + v);
      previous = current;
    }
  }

  @Test
  void higherExponentialGainSoftensTheMidRange() {
    // More cubic weighting means less output for the same small deflection.
    double soft = Utils.applyAxisFilter(0.4, 0.05, 1.0);
    double linear = Utils.applyAxisFilter(0.4, 0.05, 0.0);
    assertTrue(soft < linear, "expected the cubic curve to sit under the linear one");
  }

  @Test
  void matchesTheCppFormulaExactly() {
    double axis = 0.6;
    double deadzone = 0.05;
    double gain = 0.5;

    double normalized = (Math.abs(axis) - deadzone) / (1 - deadzone);
    double expected = gain * Math.pow(normalized, 3) + (1 - gain) * normalized;

    assertEquals(expected, Utils.applyAxisFilter(axis, deadzone, gain), kEpsilon);
  }

  @Test
  void defaultOverloadsMatchTheCppDefaultArguments() {
    assertEquals(Utils.applyAxisFilter(0.5, 0.05, 0.5), Utils.applyAxisFilter(0.5), kEpsilon);
    assertEquals(Utils.applyAxisFilter(0.5, 0.2, 0.5), Utils.applyAxisFilter(0.5, 0.2), kEpsilon);
  }
}
