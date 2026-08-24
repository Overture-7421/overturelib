// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.simulation.mechanisms;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.overture.lib.motorcontrollers.OverTalonFX;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import java.util.ArrayList;
import java.util.List;

/**
 * A simulated elevator riding on a drum, with hard stops and gravity.
 *
 * <p>A cascading elevator is drawn as a set of stages that share the travel: with three stages, the
 * first moves a third of the way, the second two thirds, and the carriage the whole distance. Only
 * the drawing is staged; the physics is one carriage, because that is what the motor feels.
 */
public class SimElevator extends SimMechanism {
  private final ElevatorSim elevatorSim;
  private final Translation3d slideAxis;
  private final double metersToRotorRotations;
  private final int stageCount;

  /**
   * Constructs a simulated elevator.
   *
   * @param robotToElevator where the elevator's bottom sits relative to the robot origin
   * @param slideAxis which way the carriage travels, as a unit vector in the elevator's own frame,
   *     so a tilted mount carries its rails with it
   * @param gearbox the motor driving it
   * @param gearing reduction from motor to drum, greater than one for a reduction
   * @param carriageMass everything the motor has to lift
   * @param drumRadius radius of the drum the belt or rope wraps around
   * @param minHeight the hard stop at the bottom
   * @param maxHeight the hard stop at the top
   * @param startingHeight where the carriage begins, between the two stops
   * @param simulateGravity whether gravity pulls on the carriage
   * @param stageCount how many stages to draw, one for a single stage elevator
   * @param motor the real motor whose simulation state drives and is driven by this
   */
  public SimElevator(
      Transform3d robotToElevator,
      Translation3d slideAxis,
      DCMotor gearbox,
      double gearing,
      Mass carriageMass,
      Distance drumRadius,
      Distance minHeight,
      Distance maxHeight,
      Distance startingHeight,
      boolean simulateGravity,
      int stageCount,
      OverTalonFX motor) {
    super(robotToElevator, motor);

    // Same reasoning as the arm: three heights in a row are easy to swap, and the symptom is a
    // carriage that will not move rather than an obvious mistake.
    if (minHeight.gte(maxHeight)) {
      throw new IllegalArgumentException(
          "Elevator minHeight ("
              + minHeight.in(Meters)
              + " m) must be below maxHeight ("
              + maxHeight.in(Meters)
              + " m)");
    }
    if (startingHeight.lt(minHeight) || startingHeight.gt(maxHeight)) {
      throw new IllegalArgumentException(
          "Elevator startingHeight ("
              + startingHeight.in(Meters)
              + " m) must be between minHeight ("
              + minHeight.in(Meters)
              + " m) and maxHeight ("
              + maxHeight.in(Meters)
              + " m)");
    }
    if (stageCount < 1) {
      throw new IllegalArgumentException(
          "Elevator stageCount must be at least 1, got " + stageCount);
    }

    this.slideAxis = slideAxis;
    this.stageCount = stageCount;
    this.elevatorSim =
        new ElevatorSim(
            gearbox,
            gearing,
            carriageMass.in(Kilograms),
            drumRadius.in(Meters),
            minHeight.in(Meters),
            maxHeight.in(Meters),
            simulateGravity,
            startingHeight.in(Meters));

    // One drum turn pays out one circumference of belt, and the rotor turns `gearing` times per
    // drum turn.
    this.metersToRotorRotations = gearing / (2.0 * Math.PI * drumRadius.in(Meters));
  }

  /**
   * Returns how high the carriage is.
   *
   * @return the carriage height
   */
  public Distance getPosition() {
    return Meters.of(elevatorSim.getPositionMeters());
  }

  /**
   * Returns how fast the carriage is moving.
   *
   * @return the carriage speed
   */
  public LinearVelocity getVelocity() {
    return MetersPerSecond.of(elevatorSim.getVelocityMetersPerSecond());
  }

  @Override
  public void update() {
    elevatorSim.setInputVoltage(getAppliedVolts());
    elevatorSim.update(getTimeStep());

    setRotorState(
        Rotations.of(elevatorSim.getPositionMeters() * metersToRotorRotations),
        RotationsPerSecond.of(elevatorSim.getVelocityMetersPerSecond() * metersToRotorRotations));
  }

  @Override
  public Current getStatorCurrent() {
    return Amps.of(elevatorSim.getCurrentDrawAmps());
  }

  /**
   * Returns one pose per stage, innermost first, with the carriage last.
   *
   * <p>Note for anyone comparing against the archived standalone simulator: that one listed the
   * carriage first. Ordering matters only for matching the list up with the components in an
   * AdvantageScope model.
   */
  @Override
  public List<Pose3d> getPoses3d() {
    double height = elevatorSim.getPositionMeters();
    List<Pose3d> poses = new ArrayList<>(stageCount);

    for (int stage = 1; stage <= stageCount; stage++) {
      double stageHeight = height * stage / stageCount;
      poses.add(
          new Pose3d()
              .transformBy(getRobotToMechanism())
              .transformBy(new Transform3d(slideAxis.times(stageHeight), new Rotation3d())));
    }

    return poses;
  }
}
