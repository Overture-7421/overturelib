// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.subsystems.leds.effects;

import com.overture.lib.subsystems.leds.LedsManager;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;

/** Pulses a strip between off and a solid color on a cosine curve. */
public class BlinkEffect extends Command {
  private final AddressableLEDBufferView ledStrip;
  private final Color8Bit color;
  private final double period;
  private double startTime;

  /**
   * Constructs a BlinkEffect with a one second period that requires the LED subsystem.
   *
   * @param leds the LED manager owning the strip
   * @param name the name of the strip slice to drive
   * @param color the color to pulse
   */
  public BlinkEffect(LedsManager leds, String name, Color8Bit color) {
    this(leds, name, color, 1.0, true);
  }

  /**
   * Constructs a BlinkEffect that requires the LED subsystem.
   *
   * @param leds the LED manager owning the strip
   * @param name the name of the strip slice to drive
   * @param color the color to pulse
   * @param period the pulse period, in seconds
   */
  public BlinkEffect(LedsManager leds, String name, Color8Bit color, double period) {
    this(leds, name, color, period, true);
  }

  /**
   * Constructs a BlinkEffect.
   *
   * @param leds the LED manager owning the strip
   * @param name the name of the strip slice to drive
   * @param color the color to pulse
   * @param period the pulse period, in seconds
   * @param addRequirement whether this command requires the LED subsystem
   */
  public BlinkEffect(
      LedsManager leds, String name, Color8Bit color, double period, boolean addRequirement) {
    this.ledStrip = leds.getLedStrip(name);
    this.color = color;
    this.period = period;

    if (addRequirement) {
      addRequirements(leds);
    }
  }

  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {
    double elapsedTime = Timer.getFPGATimestamp() - startTime;
    double blinkMultiplier = Math.cos(elapsedTime * 2.0 * Math.PI * 1.0 / period) * 0.5 + 0.5;

    int red = (int) (color.red * blinkMultiplier);
    int green = (int) (color.green * blinkMultiplier);
    int blue = (int) (color.blue * blinkMultiplier);

    for (int i = 0; i < ledStrip.getLength(); i++) {
      ledStrip.setRGB(i, red, green, blue);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
