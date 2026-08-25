// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.simulation.mechanisms;

import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.overture.lib.motorcontrollers.OverTalonFX;
import com.overture.lib.utils.Logging;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;
import org.ironmaple.simulation.motorsims.SimulatedBattery;

/**
 * A simulated mechanism bolted to the robot.
 *
 * <p>Subclasses own a WPILib physics model. This class owns the wiring to the real motor: it reads
 * the voltage the TalonFX is commanding, and takes back the position and speed the physics
 * produced. Because the voltage comes from the motor's own simulation state, whatever the TalonFX
 * is actually doing, closed loop or open, is what moves the mechanism. That matters most for the
 * case worth catching: a loop that is unstable on the robot is unstable here too, where a stand-in
 * PID controller would quietly behave itself.
 *
 * <p>Nothing registers with a manager. The subsystem that owns the mechanism calls {@link
 * #update()} from its own {@code simulationPeriodic()}, which is the hook WPILib already runs only
 * in simulation, so on a real robot none of this is ever constructed.
 */
public abstract class SimMechanism {
  private final OverTalonFX motor;
  private Transform3d robotToMechanism;
  private double lastTimestamp;

  /**
   * Constructs a mechanism at a fixed place on the robot.
   *
   * @param robotToMechanism where the mechanism sits relative to the robot origin
   * @param motor the real motor whose simulation state drives and is driven by this
   */
  protected SimMechanism(Transform3d robotToMechanism, OverTalonFX motor) {
    this.robotToMechanism = robotToMechanism;
    this.motor = motor;
    this.lastTimestamp = Timer.getFPGATimestamp();

    // A physics model's positive direction is the real one: up is up, and gravity does not care how
    // the motor is wired. Orientation makes both halves of the bridge speak the mechanism's frame,
    // so an arm on an inverted motor rises on a positive request instead of falling while its
    // encoder claims it went up.
    //
    // Phoenix documents Orientation as describing the mechanical linkage and warns against setting
    // it from the invert. The two are the same fact on a robot configured the usual way: a motor is
    // marked inverted precisely because it drives its mechanism backwards. If a mechanism ever
    // simulates falling when it should rise, this is the line -- the invert was set for some reason
    // other than the mounting.
    motor.getSimState().Orientation =
        motor.getOverConfig().Inverted
            ? ChassisReference.Clockwise_Positive
            : ChassisReference.CounterClockwise_Positive;
  }

  /**
   * Returns how long it has been since the last call, for stepping a physics model.
   *
   * <p>Measured rather than assumed to be the robot period, because the simulation loop overruns
   * often enough that a fixed step makes a mechanism run slow whenever the rest of the code is
   * busy.
   *
   * @return the elapsed time, in seconds
   */
  protected final double getTimeStep() {
    double now = Timer.getFPGATimestamp();
    double step = now - lastTimestamp;
    lastTimestamp = now;
    return step;
  }

  /**
   * Returns the voltage the motor is applying, in volts, ready to feed a physics model.
   *
   * @return the applied voltage
   */
  protected final double getAppliedVolts() {
    TalonFXSimState sim = motor.getSimState();
    // maple-sim owns the battery model, and asking it directly works before the arena has stepped
    // even once. The arena does mirror its result onto RoboRIO input voltage every sub-tick, but
    // only once it is running, and a mechanism can be spinning before then.
    sim.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
    return sim.getMotorVoltage();
  }

  /**
   * Hands the physics result back to the motor.
   *
   * <p>Both arguments are rotor side, before the gearbox, because that is what a TalonFX measures.
   * A mechanism-side value here is a factor-of-gearing error that presents as a badly tuned loop
   * rather than as a unit mistake.
   *
   * @param rotorPosition the rotor position
   * @param rotorVelocity the rotor speed
   */
  protected final void setRotorState(Angle rotorPosition, AngularVelocity rotorVelocity) {
    TalonFXSimState sim = motor.getSimState();
    sim.setRawRotorPosition(rotorPosition);
    sim.setRotorVelocity(rotorVelocity);
  }

  /**
   * Moves the mechanism's mounting point, for something carried by another mechanism.
   *
   * @param robotToMechanism the new transform
   */
  public final void setRobotToMechanism(Transform3d robotToMechanism) {
    this.robotToMechanism = robotToMechanism;
  }

  /**
   * Returns where the mechanism sits relative to the robot.
   *
   * @return the transform
   */
  public final Transform3d getRobotToMechanism() {
    return robotToMechanism;
  }

  /**
   * Logs every moving part of a set of mechanisms as one pose array, for a 3D field view.
   *
   * <p>AdvantageScope draws a robot model's components from a single array, so the mechanisms have
   * to be flattened into one signal in a fixed order. The archived standalone simulator did this in
   * its per-robot class; there is no such class any more, so the robot calls this once with the
   * mechanisms it owns.
   *
   * <p>Log it once at startup with an all-identity array of the same length too: that is the
   * "zeroed" reference AdvantageScope wants while a model is being set up.
   *
   * @param path where to log, for example {@code "/Components/Poses"}
   * @param mechanisms the mechanisms to draw, in the order the model expects them
   */
  public static void logPoses(String path, SimMechanism... mechanisms) {
    List<Pose3d> poses = new ArrayList<>();
    for (SimMechanism mechanism : mechanisms) {
      poses.addAll(mechanism.getPoses3d());
    }

    Logging.logPoses(path, poses.toArray(Pose3d[]::new), Logging.Destination.LOG_ONLY);
  }

  /** Steps the physics and pushes the result into the motor. Call once per loop. */
  public abstract void update();

  /**
   * Returns the current the mechanism is pulling through its motor windings.
   *
   * <p>Deliberately not wired into the simulated battery. This is stator current, several times
   * what the battery actually supplies, and the two quantities available here are both wrong for
   * that job: Phoenix simulates supply current but reports it negative on any mechanism whose
   * gearbox holds more motors than the one TalonFX being modelled, which is every elevator with a
   * follower. A mechanism load the drivetrain can feel is worth having one day; a made up voltage
   * sag is not.
   *
   * @return the stator current
   */
  public abstract Current getStatorCurrent();

  /**
   * Returns the poses of the mechanism's moving parts, for a 3D field view.
   *
   * @return the component poses
   */
  public abstract List<Pose3d> getPoses3d();
}
