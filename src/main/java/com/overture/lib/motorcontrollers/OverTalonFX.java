package com.overture.lib.motorcontrollers;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.overture.lib.sensors.OverCANCoder;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;

/** The OverTalonFX class is a wrapper for the TalonFX class from the CTRE Phoenix library. */
public class OverTalonFX extends TalonFX {

  private TalonFXConfiguration ctreConfig;

  /**
   * Constructor for OverTalonFX, initializes the TalonFX with the given config.
   *
   * @param overConfig The OverTalonFXConfig to initialize the TalonFX with.
   * @param bus The bus the TalonFX is connected to.
   */
  @SuppressWarnings("static-access")
  public OverTalonFX(OverTalonFXConfig overConfig, String bus) {
    super(overConfig.MotorId, bus);
    this.ctreConfig = new TalonFXConfiguration();

    this.ctreConfig
        .MotorOutput
        .withNeutralMode(overConfig.NeutralMode)
        .withInverted(overConfig.Inverted);

    this.ctreConfig.Voltage.withPeakForwardVoltage(12).withPeakReverseVoltage(-12);

    this.ctreConfig.OpenLoopRamps.withVoltageOpenLoopRampPeriod(overConfig.OpenLoopRampRate);
    this.ctreConfig.ClosedLoopRamps.withVoltageClosedLoopRampPeriod(overConfig.ClosedLoopRampRate);

    this.ctreConfig
        .CurrentLimits
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(overConfig.StatorCurrentLimit)
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLowerLimit(overConfig.CurrentLimit)
        .withSupplyCurrentLimit(overConfig.TriggerThreshold)
        .withSupplyCurrentLowerTime(overConfig.TriggerThresholdTime);

    this.ctreConfig.Slot0.from(overConfig.PIDConfigs);

    this.getConfigurator().apply(this.ctreConfig);
  }

  /**
   * Constructor for OverTalonFX, initializes the TalonFX with the given config.
   *
   * @param overConfig The OverTalonFXConfig to initialize the TalonFX with.
   */
  public OverTalonFX(OverTalonFXConfig overConfig) {
    this(overConfig, "");
  }

  /**
   * Sets the sensor to mechanism ratio of the TalonFX
   *
   * @param gearRatio The gear ratio of the TalonFX
   */
  public void setSensorToMechanism(double gearRatio) {
    this.ctreConfig.Feedback.withSensorToMechanismRatio(gearRatio);
    this.getConfigurator().apply(this.ctreConfig);
  }

  /**
   * Sets the rotor to sensor ratio of the TalonFX
   *
   * @param gearRatio The gear ratio of the TalonFX
   */
  public void setRotorToSensorRatio(double gearRatio) {
    this.ctreConfig.Feedback.withRotorToSensorRatio(gearRatio);
    this.getConfigurator().apply(this.ctreConfig);
  }

  /**
   * Sets the remote CANCoder of the TalonFX
   *
   * @param canCoder The remote CANCoder of the TalonFX
   */
  public void setRemoteCANCoder(OverCANCoder canCoder) {
    this.ctreConfig.Feedback.withRemoteCANcoder(canCoder);
    this.getConfigurator().apply(this.ctreConfig);
  }

  /**
   * Sets the fused CANCoder of the TalonFX
   *
   * @param canCoder The fused CANCoder of the TalonFX
   */
  public void setFusedCANCoder(OverCANCoder canCoder) {
    this.ctreConfig.Feedback.withFusedCANcoder(canCoder);
    this.getConfigurator().apply(this.ctreConfig);
  }

  /**
   * Sets the closed loop ramp rate of the TalonFX
   *
   * @param ramp The closed loop ramp rate of the TalonFX in seconds
   */
  public void setClosedLoopTorqueRamp(Time ramp) {
    this.ctreConfig.ClosedLoopRamps.withTorqueClosedLoopRampPeriod(ramp);
    this.getConfigurator().apply(this.ctreConfig);
  }

  /**
   * Sets the torque current limit of the TalonFX
   *
   * @param peakForward The peak forward torque current of the TalonFX in Amps
   * @param peakBackward The peak backward torque current of the TalonFX in Amps
   * @param deadband The deadband of the TalonFX in Amps
   */
  public void setTorqueCurrentLimit(Current peakForward, Current peakBackward, Current deadband) {
    this.ctreConfig
        .TorqueCurrent
        .withPeakForwardTorqueCurrent(peakForward)
        .withPeakReverseTorqueCurrent(peakBackward)
        .withTorqueNeutralDeadband(deadband);
    this.getConfigurator().apply(this.ctreConfig);
  }

  /**
   * Sets the TalonFX to follow another TalonFX
   *
   * @param masterId The ID of the TalonFX to follow
   * @param invert Whether or not to invert the following
   */
  public void setFollow(int masterId, boolean invert) {
    super.setControl(new Follower(masterId, invert));
  }

  /**
   * Gets the TalonFXConfiguration of the TalonFX
   *
   * @return The TalonFXConfiguration of the TalonFX
   */
  public TalonFXConfiguration getCtreConfig() {
    return this.ctreConfig;
  }

  /**
   * Sets the TalonFX Motion Magic values
   *
   * @param cruiseVelocity The cruise velocity of the TalonFX in Rotations per Second
   * @param acceleration The acceleration of the TalonFX in Rotations per Second Squared
   * @param jerk The jerk of the TalonFX in Rotations per Second Cubed
   */
  public void configureMotionMagic(
      AngularVelocity cruiseVelocity,
      AngularAcceleration acceleration,
      Velocity<AngularAccelerationUnit> jerk) {
    this.ctreConfig
        .MotionMagic
        .withMotionMagicCruiseVelocity(cruiseVelocity)
        .withMotionMagicAcceleration(acceleration)
        .withMotionMagicJerk(jerk);

    this.getConfigurator().apply(this.ctreConfig);
  }

  /**
   * Sets the TalonFX Motion Magic values
   *
   * <p>Jerk is defaulted to 0
   *
   * @param cruiseVelocity The cruise velocity of the TalonFX in Rotations per Second
   * @param acceleration The acceleration of the TalonFX in Rotations per Second Squared
   */
  public void configureMotionMagic(
      AngularVelocity cruiseVelocity, AngularAcceleration acceleration) {
    this.ctreConfig
        .MotionMagic
        .withMotionMagicCruiseVelocity(cruiseVelocity)
        .withMotionMagicAcceleration(acceleration);

    this.getConfigurator().apply(this.ctreConfig);
  }

  /**
   * Configures the TalonFX software limit switch
   *
   * @param configs The software limit switch configuration
   */
  public void configureSofwareLimitSwitch(SoftwareLimitSwitchConfigs configs) {
    this.ctreConfig.withSoftwareLimitSwitch(configs);
    this.getConfigurator().apply(this.ctreConfig);
  }

  /** Sets the TalonFX continuous wrap mode */
  public void setContinuousWrap() {
    this.ctreConfig.ClosedLoopGeneral.ContinuousWrap = true;
    this.getConfigurator().apply(this.ctreConfig);
  }

  /**
   * Sets the TalonFX position update frequency
   *
   * @param frequencyHz The frequency to set the TalonFX to in Hertz
   */
  public void setPositionUpdateFrequency(Frequency frequencyHz) {
    super.getPosition().setUpdateFrequency(frequencyHz);
  }

  /**
   * Sets the TalonFX velocity update frequency
   *
   * @param frequencyHz The frequency to set the TalonFX to in Hertz
   */
  public void setVelocityUpdateFrequency(Frequency frequencyHz) {
    super.getVelocity().setUpdateFrequency(frequencyHz);
  }
}
