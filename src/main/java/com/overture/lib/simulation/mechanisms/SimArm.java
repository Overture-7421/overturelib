// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.simulation.mechanisms;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.overture.lib.motorcontrollers.OverTalonFX;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import java.util.List;

/**
 * A simulated arm swinging about a single joint, with hard stops and gravity.
 *
 * <p>An absolute sensor is optional, and comes in through {@link SimEncoder} so a CANcoder and a
 * through-bore encoder on a DIO port are the same code. Leave it out and the arm is measured
 * through the motor's rotor and the gear ratio, as an arm with no absolute sensor is on the robot.
 */
public class SimArm extends SimMechanism {
  private final SingleJointedArmSim armSim;
  private final double gearing;
  private final Rotation3d rotationAxis;
  private final SimEncoder encoder;

  /**
   * Constructs a simulated arm measured through the motor's own rotor.
   *
   * @param robotToArm where the joint sits relative to the robot origin
   * @param rotationAxis which axis the arm swings about, as a unit vector in the arm's own frame
   * @param gearbox the motor driving it
   * @param gearing reduction from motor to joint, greater than one for a reduction
   * @param momentOfInertia the arm's moment of inertia about the joint, in kilogram square meters
   * @param armLength distance from the joint to the arm's center of mass
   * @param minAngle the hard stop on the way down
   * @param maxAngle the hard stop on the way up
   * @param startingAngle where the arm begins, between the two stops
   * @param simulateGravity whether gravity pulls on the arm
   * @param motor the real motor whose simulation state drives and is driven by this
   */
  public SimArm(
      Transform3d robotToArm,
      Rotation3d rotationAxis,
      DCMotor gearbox,
      double gearing,
      double momentOfInertia,
      Distance armLength,
      Angle minAngle,
      Angle maxAngle,
      Angle startingAngle,
      boolean simulateGravity,
      OverTalonFX motor) {
    this(
        robotToArm,
        rotationAxis,
        gearbox,
        gearing,
        momentOfInertia,
        armLength,
        minAngle,
        maxAngle,
        startingAngle,
        simulateGravity,
        motor,
        null);
  }

  /**
   * Constructs a simulated arm with an absolute encoder on the joint.
   *
   * @param robotToArm where the joint sits relative to the robot origin
   * @param rotationAxis which axis the arm swings about, as a unit vector in the arm's own frame
   * @param gearbox the motor driving it
   * @param gearing reduction from motor to joint, greater than one for a reduction
   * @param momentOfInertia the arm's moment of inertia about the joint, in kilogram square meters
   * @param armLength distance from the joint to the arm's center of mass
   * @param minAngle the hard stop on the way down
   * @param maxAngle the hard stop on the way up
   * @param startingAngle where the arm begins, between the two stops
   * @param simulateGravity whether gravity pulls on the arm
   * @param motor the real motor whose simulation state drives and is driven by this
   * @param encoder the absolute sensor on the joint, or null if the arm is measured through the
   *     motor, built with {@link SimEncoder#of}
   */
  public SimArm(
      Transform3d robotToArm,
      Rotation3d rotationAxis,
      DCMotor gearbox,
      double gearing,
      double momentOfInertia,
      Distance armLength,
      Angle minAngle,
      Angle maxAngle,
      Angle startingAngle,
      boolean simulateGravity,
      OverTalonFX motor,
      SimEncoder encoder) {
    super(robotToArm, motor);

    // Three angles in a row are easy to hand over in the wrong order, and the symptom is an arm
    // that refuses to move rather than an obvious mistake, so say so at construction instead.
    if (minAngle.gte(maxAngle)) {
      throw new IllegalArgumentException(
          "Arm minAngle ("
              + minAngle.in(Degrees)
              + " deg) must be below maxAngle ("
              + maxAngle.in(Degrees)
              + " deg)");
    }
    if (startingAngle.lt(minAngle) || startingAngle.gt(maxAngle)) {
      throw new IllegalArgumentException(
          "Arm startingAngle ("
              + startingAngle.in(Degrees)
              + " deg) must be between minAngle ("
              + minAngle.in(Degrees)
              + " deg) and maxAngle ("
              + maxAngle.in(Degrees)
              + " deg)");
    }

    this.gearing = gearing;
    this.rotationAxis = rotationAxis;
    this.encoder = encoder;
    this.armSim =
        new SingleJointedArmSim(
            gearbox,
            gearing,
            momentOfInertia,
            armLength.in(Meters),
            minAngle.in(Radians),
            maxAngle.in(Radians),
            simulateGravity,
            startingAngle.in(Radians));
  }

  /**
   * Returns where the arm is.
   *
   * @return the joint angle
   */
  public Angle getAngle() {
    return Radians.of(armSim.getAngleRads());
  }

  /**
   * Returns how fast the arm is swinging.
   *
   * @return the joint speed
   */
  public AngularVelocity getVelocity() {
    return RadiansPerSecond.of(armSim.getVelocityRadPerSec());
  }

  @Override
  public void update() {
    armSim.setInputVoltage(getAppliedVolts());
    armSim.update(getTimeStep());

    Angle angle = getAngle();
    AngularVelocity velocity = getVelocity();

    setRotorState(angle.times(gearing), velocity.times(gearing));

    if (encoder != null) {
      // The sensor sits on the joint, so it gets the mechanism side of the gearbox while the motor
      // gets the rotor side.
      encoder.update(angle, velocity);
    }
  }

  @Override
  public Current getStatorCurrent() {
    return Amps.of(armSim.getCurrentDrawAmps());
  }

  @Override
  public List<Pose3d> getPoses3d() {
    double angleRadians = armSim.getAngleRads();
    Rotation3d swing =
        new Rotation3d(
            rotationAxis.getX() * angleRadians,
            rotationAxis.getY() * angleRadians,
            rotationAxis.getZ() * angleRadians);

    return List.of(
        new Pose3d()
            .transformBy(getRobotToMechanism())
            .transformBy(new Transform3d(new Translation3d(), swing)));
  }
}
