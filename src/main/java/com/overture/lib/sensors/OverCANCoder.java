package com.overture.lib.sensors;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

/**
 * OverCANCoder is a wrapper class for the CTRE CANCoder that allows for more configuration options.
 */
public class OverCANCoder extends CANcoder {
  private CANcoderConfiguration ctreConfig;

  /**
   * Constructor for OverCANCoder, initializes the CANcoder with the given config.
   *
   * @param config The OverCANCoderConfig to initialize the CANcoder with.
   * @param bus The bus the CANcoder is connected to.
   */
  public OverCANCoder(OverCANCoderConfig config, String bus) {
    super(config.CanCoderId, bus);
    this.ctreConfig = new CANcoderConfiguration();

    this.ctreConfig
        .MagnetSensor
        .withSensorDirection(config.SensorDirection)
        .withMagnetOffset(config.Offset)
        .withAbsoluteSensorDiscontinuityPoint(0.5);

    this.getConfigurator().apply(this.ctreConfig);

	// TODO: Implement Simulation for OverCANCoder
  }

  /**
   * Constructor for OverCANCoder, initializes the CANcoder with the given config.
   *
   * @param config The OverCANCoderConfig to initialize the CANcoder with.
   */
  public OverCANCoder(OverCANCoderConfig config) {
    this(config, "");
  }

  /**
   * Gets the CANcoder configuration
   *
   * @return The CANcoder configuration
   */
  public CANcoderConfiguration getConfig() {
    return this.ctreConfig;
  }
}
