package com.overture.lib.Robots;

/**
 * The OverContainer interface
 *
 * <p>OverContainer is an interface that contains the methods for configuring the driver, operator,
 * default, and characterization bindings.
 */
public interface OverContainer {

  /** ConfigDriverBindings is a method that configures the driver bindings. */
  public void ConfigDriverBindings();

  /** ConfigOperatorBindings is a method that configures the operator bindings. */
  public void ConfigOperatorBindings();

  /** ConfigDefaultBindings is a method that configures the default bindings. */
  public void ConfigDefaultBindings();

  /** ConfigCharacterizationBindings is a method that configures the characterization bindings. */
  public void ConfigCharacterizationBindings();
}
