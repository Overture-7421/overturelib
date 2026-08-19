// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.gamepads;

import com.overture.lib.math.Utils;
import com.overture.lib.utils.Logging;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** An Xbox controller with deadzone filtering, combined-button triggers and telemetry. */
public class OverXboxController extends CommandXboxController {
  private final double stickDeadzone;
  private final double triggerDeadzone;

  private final Alert isConnectedAlert =
      new Alert("Controllers", "A Xbox Controller is not connected", Alert.AlertType.kWarning);

  /**
   * Constructs an OverXboxController.
   *
   * @param port the driver station port the controller is plugged into
   * @param stickDeadzone the deadzone applied to the sticks
   * @param triggerDeadzone the threshold at which a trigger counts as pressed
   */
  public OverXboxController(int port, double stickDeadzone, double triggerDeadzone) {
    super(port);
    this.stickDeadzone = stickDeadzone;
    this.triggerDeadzone = triggerDeadzone;
  }

  /**
   * Returns the twist commanded by the triggers, counter-clockwise positive.
   *
   * @return the twist, from -1 to 1
   */
  public double getTwist() {
    // Takes into account counter-clockwise twist as positive and clockwise twist as negative
    double right = getRightTriggerAxis();
    double left = getLeftTriggerAxis();
    return left - right;
  }

  /**
   * Returns the direction the left stick points in.
   *
   * @return the left stick direction, or zero when the stick is centered
   */
  public Rotation2d getLeftStickDirection() {
    double x = Utils.applyAxisFilter(-getLeftY(), stickDeadzone);
    double y = Utils.applyAxisFilter(-getLeftX(), stickDeadzone);

    if (Math.hypot(x, y) < 1e-6) {
      return Rotation2d.fromDegrees(0);
    }

    return new Rotation2d(x, y);
  }

  /**
   * Returns the direction the right stick points in.
   *
   * @return the right stick direction, or zero when the stick is centered
   */
  public Rotation2d getRightStickDirection() {
    double x = Utils.applyAxisFilter(-getRightY(), stickDeadzone);
    double y = Utils.applyAxisFilter(-getRightX(), stickDeadzone);

    if (Math.hypot(x, y) < 1e-6) {
      return Rotation2d.fromDegrees(0);
    }

    return new Rotation2d(x, y);
  }

  /**
   * Returns a trigger for the left bumper being pressed on its own.
   *
   * @return the trigger
   */
  public Trigger leftBumperOnly() {
    return leftBumper().and(rightBumper().negate());
  }

  /**
   * Returns a trigger for the right bumper being pressed on its own.
   *
   * @return the trigger
   */
  public Trigger rightBumperOnly() {
    return rightBumper().and(leftBumper().negate());
  }

  /**
   * Returns a trigger for both bumpers being pressed together.
   *
   * @return the trigger
   */
  public Trigger bothBumpers() {
    return rightBumper().and(leftBumper());
  }

  /**
   * Returns a trigger for the left trigger being pulled on its own.
   *
   * @return the trigger
   */
  public Trigger leftTriggerOnly() {
    return leftTrigger(triggerDeadzone).and(rightTrigger(triggerDeadzone).negate());
  }

  /**
   * Returns a trigger for the right trigger being pulled on its own.
   *
   * @return the trigger
   */
  public Trigger rightTriggerOnly() {
    return rightTrigger(triggerDeadzone).and(leftTrigger(triggerDeadzone).negate());
  }

  /**
   * Returns a trigger for both triggers being pulled together.
   *
   * @return the trigger
   */
  public Trigger bothTriggers() {
    return leftTrigger(triggerDeadzone).and(rightTrigger(triggerDeadzone));
  }

  /**
   * Returns a command that rumbles the controller.
   *
   * @param intensity the rumble intensity, from 0 to 1
   * @return the command
   */
  public Command getRumbleCommand(double intensity) {
    return Commands.runOnce(() -> setRumble(GenericHID.RumbleType.kBothRumble, intensity));
  }

  /**
   * Returns a trigger for the left stick being pushed past a threshold on the y axis.
   *
   * @param triggerThreshold the threshold to exceed
   * @return the trigger
   */
  public Trigger leftYTrigger(double triggerThreshold) {
    return new Trigger(() -> Math.abs(getLeftY()) >= triggerThreshold);
  }

  /**
   * Returns a trigger for the left stick being pushed past a threshold on the x axis.
   *
   * @param triggerThreshold the threshold to exceed
   * @return the trigger
   */
  public Trigger leftXTrigger(double triggerThreshold) {
    return new Trigger(() -> Math.abs(getLeftX()) >= triggerThreshold);
  }

  /**
   * Returns a trigger for the right stick being pushed past a threshold on the y axis.
   *
   * @param triggerThreshold the threshold to exceed
   * @return the trigger
   */
  public Trigger rightYTrigger(double triggerThreshold) {
    return new Trigger(() -> Math.abs(getRightY()) >= triggerThreshold);
  }

  /**
   * Returns a trigger for the right stick being pushed past a threshold on the x axis.
   *
   * @param triggerThreshold the threshold to exceed
   * @return the trigger
   */
  public Trigger rightXTrigger(double triggerThreshold) {
    return new Trigger(() -> Math.abs(getRightX()) >= triggerThreshold);
  }

  /**
   * Returns a trigger for the right stick being pushed past a threshold on either axis.
   *
   * @param triggerThreshold the threshold to exceed
   * @return the trigger
   */
  public Trigger rightStick(double triggerThreshold) {
    return new Trigger(
        () ->
            Math.abs(getRightX()) >= triggerThreshold || Math.abs(getRightY()) >= triggerThreshold);
  }

  /**
   * Returns a trigger for the left stick being pushed past a threshold on either axis.
   *
   * @param triggerThreshold the threshold to exceed
   * @return the trigger
   */
  public Trigger leftStick(double triggerThreshold) {
    return new Trigger(
        () -> Math.abs(getLeftX()) >= triggerThreshold || Math.abs(getLeftY()) >= triggerThreshold);
  }

  /** Publishes the connection state of the controller. */
  public void updateTelemetry() {
    isConnectedAlert.set(!isConnected());

    Logging.writeBoolean(
        "Controllers/XboxController-" + getHID().getPort() + "/IsConnected", isConnected());
  }
}
