// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.math;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import java.util.Map;

/**
 * An interpolating lookup table that can be populated in a single expression, the Java counterpart
 * of the C++ initializer-list constructor.
 */
public class InterpolatingTable extends InterpolatingDoubleTreeMap {
  /** Constructs an empty table. */
  public InterpolatingTable() {
    super();
  }

  /**
   * Constructs a table from a map of key/value pairs.
   *
   * @param table the entries to seed the table with
   */
  public InterpolatingTable(Map<Double, Double> table) {
    super();
    table.forEach(this::put);
  }

  /**
   * Constructs a table from an alternating list of keys and values.
   *
   * @param keysAndValues alternating key, value, key, value... entries
   * @throws IllegalArgumentException if an odd number of arguments is given
   */
  public InterpolatingTable(double... keysAndValues) {
    super();
    if (keysAndValues.length % 2 != 0) {
      throw new IllegalArgumentException(
          "InterpolatingTable requires an even number of arguments (key, value pairs)");
    }
    for (int i = 0; i < keysAndValues.length; i += 2) {
      put(keysAndValues[i], keysAndValues[i + 1]);
    }
  }
}
