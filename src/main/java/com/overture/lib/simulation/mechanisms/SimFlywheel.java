// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.simulation.mechanisms;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.overture.lib.motorcontrollers.OverTalonFX;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import java.util.List;

/** A simulated flywheel driven by a real TalonFX's own control loop. */
public class SimFlywheel extends SimMechanism {
  private final FlywheelSim flywheelSim;
  private final double gearing;
  private final Rotation3d rotationAxis;

  // FlywheelSim models speed only, so the angle is integrated here. It exists purely so the motor
  // has a position to report and the 3D view has something to spin.
  private double rotorPositionRotations;
  private double mechanismAngleRadians;

  /**
   * Constructs a simulated flywheel.
   *
   * @param robotToWheel where the wheel sits relative to the robot origin
   * @param rotationAxis which axis the wheel spins about, as a unit vector in the wheel's frame
   * @param gearbox the motor driving it
   * @param gearing reduction from motor to wheel, greater than one for a reduction
   * @param momentOfInertia the wheel's moment of inertia, in kilogram square meters
   * @param motor the real motor whose simulation state drives and is driven by this
   */
  public SimFlywheel(
      Transform3d robotToWheel,
      Rotation3d rotationAxis,
      DCMotor gearbox,
      double gearing,
      double momentOfInertia,
      OverTalonFX motor) {
    super(robotToWheel, motor);
    this.gearing = gearing;
    this.rotationAxis = rotationAxis;
    this.flywheelSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(gearbox, momentOfInertia, gearing), gearbox);
  }

  /**
   * Returns how fast the wheel is spinning.
   *
   * @return the wheel speed
   */
  public AngularVelocity getVelocity() {
    return flywheelSim.getAngularVelocity();
  }

  @Override
  public void update() {
    double timeStep = getTimeStep();

    flywheelSim.setInputVoltage(getAppliedVolts());
    flywheelSim.update(timeStep);

    double wheelRotationsPerSecond = flywheelSim.getAngularVelocity().in(RotationsPerSecond);
    double rotorRotationsPerSecond = wheelRotationsPerSecond * gearing;

    rotorPositionRotations += rotorRotationsPerSecond * timeStep;
    mechanismAngleRadians += flywheelSim.getAngularVelocityRadPerSec() * timeStep;

    setRotorState(
        Rotations.of(rotorPositionRotations), RotationsPerSecond.of(rotorRotationsPerSecond));
  }

  @Override
  public Current getStatorCurrent() {
    return Amps.of(flywheelSim.getCurrentDrawAmps());
  }

  @Override
  public List<Pose3d> getPoses3d() {
    Rotation3d spin =
        new Rotation3d(
            rotationAxis.getX() * mechanismAngleRadians,
            rotationAxis.getY() * mechanismAngleRadians,
            rotationAxis.getZ() * mechanismAngleRadians);

    return List.of(
        new Pose3d()
            .transformBy(getRobotToMechanism())
            .transformBy(new Transform3d(new Translation3d(), spin)));
  }
}
