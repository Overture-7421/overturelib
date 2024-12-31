package com.overture.lib.subsystems.LedsManager.Effects;

import static edu.wpi.first.units.Units.*;

import com.overture.lib.subsystems.LedsManager.LedsManager;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;

/** A command that blinks an LED strip with a given color. */
public class BlinkEffect extends Command {
  private final Color8Bit color;
  private final AddressableLEDBufferView ledStrip;
  private double startTime;
  private Time period;

  /**
   * Creates a new BlinkEffect.
   *
   * @param leds the LedsManager
   * @param ledStripName the name of the led strip
   * @param color the color of the effect
   * @param period the period of the effect (in seconds)
   * @param addRequirement whether to add the requirement to the LedsManager
   */
  public BlinkEffect(
      LedsManager leds, String ledStripName, Color8Bit color, Time period, boolean addRequirement) {
    this.color = color;
    this.ledStrip = leds.getLedStrip(ledStripName);
    this.period = period;

    if (addRequirement) {
      addRequirements(leds);
    }
  }

  /**
   * Creates a new BlinkEffect.
   *
   * @param leds the LedsManager
   * @param ledStripName the name of the led strip
   * @param color the color of the effect
   */
  public BlinkEffect(LedsManager leds, String ledStripName, Color8Bit color) {
    this(leds, ledStripName, color, Seconds.of(1.0), true);
  }

  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {
    double elapsedTime = Timer.getFPGATimestamp() - startTime;
    double blinkMultiplier =
        Math.cos(elapsedTime * 2.0 * Math.PI * 1.0 / period.baseUnitMagnitude()) * 0.5 + 0.5;

    double red = color.red * blinkMultiplier;
    double green = color.green * blinkMultiplier;
    double blue = color.blue * blinkMultiplier;

    for (int i = 0; i < ledStrip.getLength(); i++) {
      ledStrip.setRGB(i, (int) red, (int) green, (int) blue);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
