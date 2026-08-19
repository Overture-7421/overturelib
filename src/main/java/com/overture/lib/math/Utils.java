// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.math;

/** Small math helpers shared across the library. */
public final class Utils {
  private Utils() {}

  /**
   * Returns the sign of a value.
   *
   * @param value the value to take the sign of
   * @return 1 if positive, -1 if negative, 0 if zero
   */
  public static int sgn(double value) {
    return (int) (Math.signum(value));
  }

  /**
   * Apply a filter to an Axis, so that when the driver is not using it, the robot doesn't move
   * randomly. It applies an Exponential curve for finer control at smaller inputs.
   *
   * <p>To learn more: https://www.desmos.com/calculator/kvubon8yfw?lang=es
   *
   * @param axisValue the raw axis value given by the joystick
   * @param deadzone the threshold for when the axis is considered as valid
   * @param exponentialGain how much of an Exponential curve we want
   * @return the filtered axis value
   */
  public static double applyAxisFilter(double axisValue, double deadzone, double exponentialGain) {
    double axisMag = Math.abs(axisValue);
    if (axisMag < deadzone) {
      return 0.0;
    }

    double res =
        exponentialGain * Math.pow((axisMag - deadzone) / (1 - deadzone), 3)
            + (1 - exponentialGain) * (axisMag - deadzone) / (1 - deadzone);

    return res * sgn(axisValue);
  }

  /**
   * Applies the axis filter with the default exponential gain of 0.5.
   *
   * @param axisValue the raw axis value given by the joystick
   * @param deadzone the threshold for when the axis is considered as valid
   * @return the filtered axis value
   */
  public static double applyAxisFilter(double axisValue, double deadzone) {
    return applyAxisFilter(axisValue, deadzone, 0.5);
  }

  /**
   * Applies the axis filter with the default deadzone of 0.05 and exponential gain of 0.5.
   *
   * @param axisValue the raw axis value given by the joystick
   * @return the filtered axis value
   */
  public static double applyAxisFilter(double axisValue) {
    return applyAxisFilter(axisValue, 0.05, 0.5);
  }
}
