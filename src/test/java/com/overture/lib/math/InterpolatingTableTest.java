// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.math;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import java.util.LinkedHashMap;
import java.util.Map;
import org.junit.jupiter.api.Test;

/** Covers the constructors the C++ initializer-list table was ported into. */
class InterpolatingTableTest {
  @Test
  void varargsConstructorInterpolatesBetweenEntries() {
    InterpolatingTable table = new InterpolatingTable(1.0, 10.0, 3.0, 30.0);

    assertEquals(10.0, table.get(1.0), 1e-9);
    assertEquals(30.0, table.get(3.0), 1e-9);
    assertEquals(20.0, table.get(2.0), 1e-9, "midpoint should interpolate linearly");
  }

  @Test
  void varargsConstructorRejectsAnOddNumberOfArguments() {
    // A dangling key would otherwise be silently dropped, which on a shooter table means one
    // distance quietly falling back to whatever the neighbours interpolate to.
    assertThrows(IllegalArgumentException.class, () -> new InterpolatingTable(1.0, 10.0, 3.0));
  }

  @Test
  void mapConstructorMatchesTheVarargsOne() {
    Map<Double, Double> entries = new LinkedHashMap<>();
    entries.put(1.0, 10.0);
    entries.put(3.0, 30.0);

    InterpolatingTable fromMap = new InterpolatingTable(entries);
    InterpolatingTable fromVarargs = new InterpolatingTable(1.0, 10.0, 3.0, 30.0);

    assertEquals(fromVarargs.get(2.0), fromMap.get(2.0), 1e-9);
  }

  @Test
  void queriesOutsideTheTableClampToTheEndEntries() {
    InterpolatingTable table = new InterpolatingTable(1.0, 10.0, 3.0, 30.0);

    assertEquals(10.0, table.get(0.0), 1e-9, "below the first key should clamp, not extrapolate");
    assertEquals(30.0, table.get(9.0), 1e-9, "above the last key should clamp, not extrapolate");
  }

  @Test
  void emptyConstructorStartsEmptyAndAcceptsEntries() {
    InterpolatingTable table = new InterpolatingTable();
    table.put(0.0, 5.0);
    table.put(2.0, 15.0);

    assertEquals(10.0, table.get(1.0), 1e-9);
  }
}
