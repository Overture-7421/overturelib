package com.overture.lib.subsystems.LedsManager.Effects;

import com.overture.lib.subsystems.LedsManager.LedsManager;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;

/** A static effect that sets all LEDs to a single color. */
public class StaticEffect extends Command {
  private final Color8Bit color;
  private final AddressableLEDBufferView ledStrip;

  /**
   * Creates a new StaticEffect.
   *
   * @param leds the LedsManager
   * @param ledStripName the name of the led strip
   * @param color the color of the effect
   * @param addRequirement whether to add the requirement to the LedsManager
   */
  public StaticEffect(
      LedsManager leds, String ledStripName, Color8Bit color, boolean addRequirement) {
    this.color = color;
    this.ledStrip = leds.getLedStrip(ledStripName);

    if (addRequirement) {
      addRequirements(leds);
    }
  }

  /**
   * Creates a new StaticEffect.
   *
   * @param leds the LedsManager
   * @param ledStripName the name of the led strip
   * @param color the color of the effect
   */
  public StaticEffect(LedsManager leds, String ledStripName, Color8Bit color) {
    this(leds, ledStripName, color, true);
  }

  @Override
  public void initialize() {
    for (int i = 0; i < ledStrip.getLength(); i++) {
      ledStrip.setLED(i, color);
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
