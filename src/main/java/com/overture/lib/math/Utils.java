package com.overture.lib.math;

/**
 * The Utils class
 *
 * <p>Utils is a class that contains the ApplyAxisFilter method for filtering the axis values.
 */
public class Utils {
  /**
   * Apply a filter to an Axis, so that when the driver is not using it, the robot doesn't move
   * randomly. It applies an Exponential curve for finer control at smaller inputs.
   *
   * <p>axisValue = The raw axis value given by the joystick deadzone = The threshold for when the
   * axis is considered as valid exponentialGain = How much of an Exponential curve we want
   *
   * <p>To learn more: https://www.desmos.com/calculator/kvubon8yfw?lang=es
   *
   * @param axisValue The raw axis value given by the joystick
   * @param deadzone The threshold for when the axis is considered as valid
   * @param exponentialGain How much of an Exponential curve we want
   * @return The filtered axis value
   */
  public static double ApplyAxisFilter(double axisValue, double deadzone, double exponentialGain) {
    double axisMag = Math.abs(axisValue);

    if (axisMag < deadzone) {
      return 0;
    }

    double res =
        exponentialGain * Math.pow((axisMag - deadzone) / (1 - deadzone), 3)
            + (1 - exponentialGain) * (axisMag - deadzone) / (1 - deadzone);

    return res * Math.signum(axisValue);
  }

  /**
   * Apply a filter to an Axis, so that when the driver is not using it, the robot doesn't move
   * randomly. It applies an Exponential curve for finer control at smaller inputs.
   *
   * <p>axisValue = The raw axis value given by the joystick deadzone = The threshold for when the
   * axis is considered as valid
   *
   * <p>To learn more: https://www.desmos.com/calculator/kvubon8yfw?lang=es
   *
   * @param axisValue The raw axis value given by the joystick
   * @param deadzone The threshold for when the axis is considered as valid
   * @return The filtered axis value
   */
  public static double ApplyAxisFilter(double axisValue, double deadzone) {
    return ApplyAxisFilter(axisValue, deadzone, 0.5);
  }

  /**
   * Apply a filter to an Axis, so that when the driver is not using it, the robot doesn't move
   * randomly. It applies an Exponential curve for finer control at smaller inputs.
   *
   * <p>axisValue = The raw axis value given by the joystick
   *
   * <p>To learn more: https://www.desmos.com/calculator/kvubon8yfw?lang=es
   *
   * @param axisValue The raw axis value given by the joystick
   * @return The filtered axis value
   */
  public static double ApplyAxisFilter(double axisValue) {
    return ApplyAxisFilter(axisValue, 0.05, 0.5);
  }
}
