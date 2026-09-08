// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.motorcontrollers;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Alert;

/** A TalonFX preconfigured from a {@link TalonFXConfiguration}. */
public class OverTalonFX extends TalonFX {
  private final TalonFXConfiguration ctreConfig;

  private final Alert isConnectedAlert =
      new Alert("Devices", "TalonFX is not connected", Alert.AlertType.kError);

  /**
   * Constructs an OverTalonFX and applies the given configuration.
   *
   * <p>The configuration is cloned, matching the C++ which took and stored this struct by value.
   * {@link TalonFXConfiguration#clone()} is a deep copy down to the gain slots, so one
   * configuration may be re-stamped across several motors without a later edit reaching back into
   * an already built one.
   *
   * @param config the configuration to apply
   * @param id the CAN id of the TalonFX
   * @param bus the CAN bus the TalonFX lives on
   */
  public OverTalonFX(TalonFXConfiguration config, int id, CANBus bus) {
    super(id, bus);
    this.ctreConfig = config.clone();

    // Aplicar la configuracion
    getConfigurator().apply(ctreConfig);

    isConnectedAlert.setText("Motor " + id + " is not connected");
  }

  /**
   * Sets the sensor to mechanism ratio of the TalonFX.
   *
   * @param gearRatio the gear ratio between the sensor and the mechanism
   */
  public void setSensorToMechanism(double gearRatio) {
    ctreConfig.Feedback.withSensorToMechanismRatio(gearRatio);
    getConfigurator().apply(ctreConfig);
  }

  /**
   * Sets the rotor to sensor ratio of the TalonFX.
   *
   * @param gearRatio the gear ratio between the rotor and the sensor
   */
  public void setRotorToSensorRatio(double gearRatio) {
    ctreConfig.Feedback.withRotorToSensorRatio(gearRatio);
    getConfigurator().apply(ctreConfig);
  }

  /**
   * Uses a remote CANcoder as the feedback source.
   *
   * @param id the device id of the remote CANcoder
   */
  public void setRemoteCANCoder(int id) {
    ctreConfig
        .Feedback
        .withFeedbackRemoteSensorID(id)
        .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder);
    getConfigurator().apply(ctreConfig);
  }

  /**
   * Uses a fused CANcoder as the feedback source.
   *
   * @param id the device id of the fused CANcoder
   */
  public void setFusedCANCoder(int id) {
    ctreConfig
        .Feedback
        .withFeedbackRemoteSensorID(id)
        .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder);
    getConfigurator().apply(ctreConfig);
  }

  /**
   * Uses a synced CANcoder as the feedback source.
   *
   * @param id the device id of the sync CANcoder
   */
  public void setSyncCANCoder(int id) {
    ctreConfig
        .Feedback
        .withFeedbackRemoteSensorID(id)
        .withFeedbackSensorSource(FeedbackSensorSourceValue.SyncCANcoder);
    getConfigurator().apply(ctreConfig);
  }

  /**
   * Sets the closed loop torque ramp rate of the TalonFX.
   *
   * @param rampSeconds the ramp period, in seconds
   */
  public void setClosedLoopTorqueRamp(double rampSeconds) {
    ctreConfig.ClosedLoopRamps.withTorqueClosedLoopRampPeriod(rampSeconds);
    getConfigurator().apply(ctreConfig);
  }

  /**
   * Sets the torque current limits of the TalonFX.
   *
   * @param peakForward the peak forward torque current, in amps
   * @param peakBackward the peak reverse torque current, in amps
   * @param deadband the torque neutral deadband, in amps
   */
  public void setTorqueCurrentLimit(double peakForward, double peakBackward, double deadband) {
    ctreConfig
        .TorqueCurrent
        .withPeakForwardTorqueCurrent(peakForward)
        .withPeakReverseTorqueCurrent(peakBackward)
        .withTorqueNeutralDeadband(deadband);
    getConfigurator().apply(ctreConfig);
  }

  /**
   * Sets this TalonFX to follow another TalonFX.
   *
   * @param masterID the device id of the TalonFX to follow
   * @param inverted whether this motor opposes the direction of the leader
   */
  public void setFollow(int masterID, boolean inverted) {
    setControl(
        new Follower(
            masterID, inverted ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));
  }

  /**
   * Returns the underlying CTRE configuration.
   *
   * @return the TalonFX configuration
   */
  public TalonFXConfiguration getCTREConfig() {
    return ctreConfig;
  }

  /**
   * Configures the Motion Magic profile of the TalonFX.
   *
   * @param cruiseVelocity the cruise velocity, in rotations per second
   * @param acceleration the acceleration, in rotations per second squared
   * @param jerk the jerk, in rotations per second cubed
   */
  public void configureMotionMagic(double cruiseVelocity, double acceleration, double jerk) {
    ctreConfig
        .MotionMagic
        .withMotionMagicCruiseVelocity(cruiseVelocity)
        .withMotionMagicAcceleration(acceleration)
        .withMotionMagicJerk(jerk);

    getConfigurator().apply(ctreConfig);
  }

  /**
   * Configures the software limit switches of the TalonFX.
   *
   * @param configs the software limit switch configuration
   */
  public void configureSoftwareLimitSwitch(SoftwareLimitSwitchConfigs configs) {
    // Cloned for the same reason as the slot 0 gains: the C++ took this parameter
    // by value.
    ctreConfig.withSoftwareLimitSwitch(configs.clone());
    getConfigurator().apply(ctreConfig);
  }

  /**
   * Sets the neutral mode, keeping the stored configuration in step with the device.
   * 
   * @param mode the neutral mode to apply
   */
  public void configureNeutralMode(NeutralModeValue mode) {
    ctreConfig.MotorOutput.withNeutralMode(mode);
    getConfigurator().apply(ctreConfig.MotorOutput);
  }

  /** Makes the closed loop take the shortest path across the rotation wrap point. */
  public void setContinuousWrap() {
    ctreConfig.ClosedLoopGeneral.ContinuousWrap = true;
    getConfigurator().apply(ctreConfig);
  }

  /**
   * Sets the update frequency of the position signal.
   *
   * @param frequencyHz the frequency, in hertz
   */
  public void setPositionUpdateFrequency(double frequencyHz) {
    getPosition().setUpdateFrequency(frequencyHz);
  }

  /**
   * Sets the update frequency of the velocity signal.
   *
   * @param frequencyHz the frequency, in hertz
   */
  public void setVelocityUpdateFrequency(double frequencyHz) {
    getVelocity().setUpdateFrequency(frequencyHz);
  }

  /** Raises the disconnection alert when the device stops responding. */
  public void updateConnectionAlert() {
    isConnectedAlert.set(!isConnected());
  }
}
