package com.overture.lib.Gamepads;

import com.overture.lib.math.Utils;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * The OverXboxController class
 *
 * <p>OverXboxController is a class that extends the CommandXboxController class and adds additional
 * methods for getting the twist of the controller, the direction of the left and right sticks, and
 * the ability to get a rumble command.
 */
public class OverXboxController extends CommandXboxController {
  private double stickDeadzone;
  private double triggerDeadzone;

  /**
   * Constructor for OverXboxController
   *
   * @param port The port of the controller
   * @param stickDeadzone The deadzone of the sticks
   * @param triggerDeadzone The deadzone of the triggers
   */
  public OverXboxController(int port, double stickDeadzone, double triggerDeadzone) {
    super(port);
    this.stickDeadzone = stickDeadzone;
    this.triggerDeadzone = triggerDeadzone;
  }

  double getTwist() {
    // Takes into account counter-clockwise twist as positive and clockwise twist as
    // negative
    return getLeftTriggerAxis() - getRightTriggerAxis();
  }

  Rotation2d getLeftStickDirection() {
    double x = Utils.ApplyAxisFilter(-this.getLeftY(), stickDeadzone);
    double y = Utils.ApplyAxisFilter(this.getLeftX(), stickDeadzone);
    return new Rotation2d(x, y);
  }

  Rotation2d getRightStickDirection() {
    double x = Utils.ApplyAxisFilter(-this.getRightY(), stickDeadzone);
    double y = Utils.ApplyAxisFilter(this.getRightX(), stickDeadzone);
    return new Rotation2d(x, y);
  }

  Trigger leftTriggerOnly() {
    return leftTrigger(triggerDeadzone).and(rightTrigger(triggerDeadzone).negate());
  }

  Trigger rightTriggerOnly() {
    return rightTrigger(triggerDeadzone).and(leftTrigger(triggerDeadzone).negate());
  }

  Trigger bothTriggers() {
    return leftTrigger(triggerDeadzone).and(rightTrigger(triggerDeadzone));
  }

  Command getRumbleCommand(double intensity) {
    return Commands.runOnce(() -> setRumble(RumbleType.kBothRumble, intensity));
  }

  Trigger leftYTrigger(double triggerTreshold) {
    return new Trigger(() -> Math.abs(getLeftY()) > triggerTreshold);
  }

  Trigger rightYTrigger(double triggerTreshold) {
    return new Trigger(() -> Math.abs(getRightY()) > triggerTreshold);
  }

  Trigger leftXTrigger(double triggerTreshold) {
    return new Trigger(() -> Math.abs(getLeftX()) > triggerTreshold);
  }

  Trigger rightXTrigger(double triggerTreshold) {
    return new Trigger(() -> Math.abs(getRightX()) > triggerTreshold);
  }

  Trigger rightStick(double triggerTreshold) {
    return new Trigger(
        () -> Math.abs(getRightX()) > triggerTreshold || Math.abs(getRightY()) > triggerTreshold);
  }

  Trigger leftStick(double triggerTreshold) {
    return new Trigger(
        () -> Math.abs(getLeftX()) > triggerTreshold || Math.abs(getLeftY()) > triggerTreshold);
  }
}
