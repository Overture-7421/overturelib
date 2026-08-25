// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.subsystems.leds.effects;

import com.overture.lib.subsystems.leds.LedsManager;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;

/** Holds a strip at a single solid color. */
public class StaticEffect extends Command {
  private final AddressableLEDBufferView ledStrip;
  private final Color8Bit color;

  /**
   * Constructs a StaticEffect that requires the LED subsystem.
   *
   * @param leds the LED manager owning the strip
   * @param name the name of the strip slice to drive
   * @param color the color to hold
   */
  public StaticEffect(LedsManager leds, String name, Color8Bit color) {
    this(leds, name, color, true);
  }

  /**
   * Constructs a StaticEffect.
   *
   * @param leds the LED manager owning the strip
   * @param name the name of the strip slice to drive
   * @param color the color to hold
   * @param addRequirement whether this command requires the LED subsystem
   */
  public StaticEffect(LedsManager leds, String name, Color8Bit color, boolean addRequirement) {
    this.ledStrip = leds.getLedStrip(name);
    this.color = color;
    if (addRequirement) {
      addRequirements(leds);
    }
  }

  @Override
  public void initialize() {
    for (int i = 0; i < ledStrip.getLength(); i++) {
      ledStrip.setRGB(i, color.red, color.green, color.blue);
    }
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
