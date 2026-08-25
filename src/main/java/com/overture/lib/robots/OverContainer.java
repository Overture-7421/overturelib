// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.robots;

/** Contract every robot container implements so bindings are wired in a predictable order. */
public interface OverContainer {
  /** Configures the driver's button bindings. */
  void configDriverBindings();

  /** Configures the operator's button bindings. */
  void configOperatorBindings();

  /** Configures the default commands of every subsystem. */
  void configDefaultCommands();

  /** Configures the bindings used for characterization routines. */
  void configCharacterizationBindings();

  /** Publishes telemetry, called once per robot period. */
  void updateTelemetry();
}
