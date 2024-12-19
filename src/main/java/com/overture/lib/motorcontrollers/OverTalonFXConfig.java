package com.overture.lib.motorcontrollers;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;

/**
 * OverTalonFXConfig is a configuration class for OverTalonFX that allows for more configuration
 * options.
 */
public class OverTalonFXConfig {
  /** The motor id */
  public int MotorId;

  /** The neutral mode */
  public NeutralModeValue NeutralMode;

  /** The inverted mode */
  public InvertedValue Inverted;

  /** The use FOC mode */
  public boolean useFOC;

  /** The PID configs */
  public SlotConfigs PIDConfigs;

  /** The current limit in amps */
  public Current CurrentLimit;

  /** The stator current limit in amps */
  public Current StatorCurrentLimit;

  /** The trigger threshold in amps */
  public Current TriggerThreshold;

  /** The trigger threshold time in seconds */
  public Time TriggerThresholdTime;

  /** The closed loop ramp rate in seconds */
  public Time ClosedLoopRampRate;

  /** The open loop ramp rate in seconds */
  public Time OpenLoopRampRate;

  /** Constructor for OverTalonFXConfig */
  OverTalonFXConfig() {
    MotorId = -1;
    NeutralMode = NeutralModeValue.Brake;
    Inverted = InvertedValue.CounterClockwise_Positive;
    useFOC = false;

    PIDConfigs = new SlotConfigs();

    CurrentLimit = Amps.zero();
    StatorCurrentLimit = Amps.zero();
    TriggerThreshold = Amps.zero();
    TriggerThresholdTime = Seconds.zero();
    ClosedLoopRampRate = Seconds.zero();
    OpenLoopRampRate = Seconds.zero();
  }

  /**
   * Sets the motor id
   *
   * @param MotorId The motor id
   */
  OverTalonFXConfig withMotorId(int MotorId) {
    this.MotorId = MotorId;
    return this;
  }

  /**
   * Sets the neutral mode
   *
   * @param NeutralMode The neutral mode
   */
  OverTalonFXConfig withNeutralMode(NeutralModeValue NeutralMode) {
    this.NeutralMode = NeutralMode;
    return this;
  }

  /**
   * Sets the inverted mode
   *
   * @param Inverted The inverted mode
   */
  OverTalonFXConfig withInverted(InvertedValue Inverted) {
    this.Inverted = Inverted;
    return this;
  }

  /**
   * Sets the use FOC mode
   *
   * @param useFOC The use FOC mode
   */
  OverTalonFXConfig withUseFOC(boolean useFOC) {
    this.useFOC = useFOC;
    return this;
  }

  /**
   * Sets the PID configs
   *
   * @param PIDConfigs The PID configs
   */
  OverTalonFXConfig withPIDConfigs(SlotConfigs PIDConfigs) {
    this.PIDConfigs = PIDConfigs;
    return this;
  }

  /**
   * Sets the current limit
   *
   * @param CurrentLimit The current limit in amps
   */
  OverTalonFXConfig withCurrentLimit(double CurrentLimit) {
    this.CurrentLimit = Amps.of(CurrentLimit);
    return this;
  }

  /**
   * Sets the stator current limit
   *
   * @param StatorCurrentLimit The stator current limit in amps
   */
  OverTalonFXConfig withStatorCurrentLimit(double StatorCurrentLimit) {
    this.StatorCurrentLimit = Amps.of(StatorCurrentLimit);
    return this;
  }

  /**
   * Sets the trigger threshold
   *
   * @param TriggerThreshold The trigger threshold in amps
   */
  OverTalonFXConfig withTriggerThreshold(double TriggerThreshold) {
    this.TriggerThreshold = Amps.of(TriggerThreshold);
    return this;
  }

  /**
   * Sets the trigger threshold time
   *
   * @param TriggerThresholdTime The trigger threshold time in seconds
   */
  OverTalonFXConfig withTriggerThresholdTime(double TriggerThresholdTime) {
    this.TriggerThresholdTime = Seconds.of(TriggerThresholdTime);
    return this;
  }

  /**
   * Sets the closed loop ramp rate
   *
   * @param ClosedLoopRampRate The closed loop ramp rate in seconds
   */
  OverTalonFXConfig withClosedLoopRampRate(double ClosedLoopRampRate) {
    this.ClosedLoopRampRate = Seconds.of(ClosedLoopRampRate);
    return this;
  }

  /**
   * Sets the open loop ramp rate
   *
   * @param OpenLoopRampRate The open loop ramp rate in seconds
   */
  OverTalonFXConfig withOpenLoopRampRate(double OpenLoopRampRate) {
    this.OpenLoopRampRate = Seconds.of(OpenLoopRampRate);
    return this;
  }
}
