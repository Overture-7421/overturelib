// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.subsystems.leds.effects;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.overture.lib.subsystems.leds.LedsManager;
import com.overture.lib.subsystems.leds.SharedTestLeds;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/** Drives the LED effects and reads the resulting buffer back. */
class LedEffectsTest {
  private static final Color8Bit kOrange = new Color8Bit(255, 100, 0);
  private static final double kPeriod = 2.0;

  private static LedsManager leds;

  @BeforeAll
  static void setUp() {
    leds = SharedTestLeds.get();
  }

  @AfterEach
  void resumeTiming() {
    if (SimHooks.isTimingPaused()) {
      SimHooks.resumeTiming();
    }
  }

  @Test
  void staticEffectFillsItsSliceWithTheColor() {
    StaticEffect effect = new StaticEffect(leds, SharedTestLeds.kTarget, kOrange);
    effect.initialize();

    AddressableLEDBufferView target = leds.getLedStrip(SharedTestLeds.kTarget);
    for (int i = 0; i < target.getLength(); i++) {
      Color8Bit led = target.getLED8Bit(i);
      assertEquals(255, led.red, "red at " + i);
      assertEquals(100, led.green, "green at " + i);
      assertEquals(0, led.blue, "blue at " + i);
    }
  }

  @Test
  void staticEffectLeavesNeighbouringSlicesAlone() {
    AddressableLEDBufferView neighbour = leds.getLedStrip(SharedTestLeds.kNeighbour);
    for (int i = 0; i < neighbour.getLength(); i++) {
      neighbour.setRGB(i, 7, 8, 9);
    }

    new StaticEffect(leds, SharedTestLeds.kTarget, kOrange).initialize();

    // A view that wrote past its own slice would be a silent corruption of whatever else shares
    // the strip, which on a real robot looks like an unrelated indicator misbehaving.
    for (int i = 0; i < neighbour.getLength(); i++) {
      Color8Bit led = neighbour.getLED8Bit(i);
      assertEquals(7, led.red, "red at " + i);
      assertEquals(8, led.green, "green at " + i);
      assertEquals(9, led.blue, "blue at " + i);
    }
  }

  @Test
  void blinkEffectFollowsTheCosineOverOnePeriod() {
    BlinkEffect effect = new BlinkEffect(leds, SharedTestLeds.kTarget, kOrange, kPeriod);
    AddressableLEDBufferView target = leds.getLedStrip(SharedTestLeds.kTarget);

    // Pausing the simulated clock is what makes this deterministic: the effect reads
    // Timer.getFPGATimestamp() directly, so without it the phase depends on how fast the test ran.
    SimHooks.pauseTiming();
    effect.initialize();

    // cos(0) * 0.5 + 0.5 = 1, so full brightness at the start.
    effect.execute();
    assertEquals(255, target.getLED8Bit(0).red, "expected full brightness at t=0");
    assertEquals(100, target.getLED8Bit(0).green, "expected full brightness at t=0");

    // cos(pi) * 0.5 + 0.5 = 0, so dark at half a period.
    SimHooks.stepTiming(kPeriod / 2.0);
    effect.execute();
    assertEquals(0, target.getLED8Bit(0).red, "expected dark at half a period");
    assertEquals(0, target.getLED8Bit(0).green, "expected dark at half a period");

    // Back to full after a whole period.
    SimHooks.stepTiming(kPeriod / 2.0);
    effect.execute();
    assertEquals(255, target.getLED8Bit(0).red, "expected full brightness after one period");
    assertEquals(100, target.getLED8Bit(0).green, "expected full brightness after one period");
  }

  @Test
  void blinkEffectIsHalfBrightAtAQuarterPeriod() {
    BlinkEffect effect = new BlinkEffect(leds, SharedTestLeds.kTarget, kOrange, kPeriod);
    AddressableLEDBufferView target = leds.getLedStrip(SharedTestLeds.kTarget);

    SimHooks.pauseTiming();
    effect.initialize();
    SimHooks.stepTiming(kPeriod / 4.0);
    effect.execute();

    // cos(pi/2) * 0.5 + 0.5 = 0.5. Pinning a midpoint as well as the endpoints is what
    // distinguishes the cosine from a square wave that happens to agree at 0 and half period.
    assertEquals(127.0, target.getLED8Bit(0).red, 1.0, "half brightness at a quarter period");
    assertEquals(50.0, target.getLED8Bit(0).green, 1.0, "half brightness at a quarter period");
  }
}
