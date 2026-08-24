// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.simulation.swerve;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import edu.wpi.first.math.geometry.Pose2d;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

/**
 * Runs a maple-sim drivetrain in the same process as the robot code.
 *
 * <p>This used to be a separate program talking over NetworkTables, because the library was C++ and
 * maple-sim is Java only. NetworkTables was the language bridge, not a design choice, and it cost
 * roughly 100 ms of lag, a startup race that silently corrupted runs, and timestamps in the wrong
 * time base. With both sides in Java the physics can simply run here.
 *
 * <p>Nothing constructs this on a real robot: {@link com.overture.lib.subsystems.swerve.SwerveBase}
 * only builds it when {@code RobotBase.isSimulation()} is true and the drivetrain has supplied a
 * simulation config, so the maple-sim classes are never touched on the roboRIO.
 */
public class SimSwerveDrivetrain {
  private final SwerveDriveSimulation drivetrain;
  private final Pigeon2SimState pigeonSim;

  /**
   * Builds the simulated drivetrain and registers it with the arena.
   *
   * @param config the drivetrain the robot is simulating, supplied by the robot project because
   *     gear ratios and module positions belong beside the real ones
   * @param startingPose where the robot starts on the field
   * @param pigeon the gyro to drive from the simulated one, or null to leave it alone
   */
  public SimSwerveDrivetrain(
      DriveTrainSimulationConfig config, Pose2d startingPose, Pigeon2 pigeon) {
    drivetrain = new SwerveDriveSimulation(config, startingPose);
    pigeonSim = pigeon == null ? null : pigeon.getSimState();
    SimulatedArena.getInstance().addDriveTrainSimulation(drivetrain);
  }

  /**
   * Returns one of the four simulated modules.
   *
   * <p>The order matches the module translations the config was built with, which is the same order
   * the kinematics use: front left, front right, back left, back right.
   *
   * @param index the module index, 0 to 3
   * @return the simulated module
   */
  public SwerveModuleSimulation getModule(int index) {
    return drivetrain.getModules()[index];
  }

  /**
   * Steps the physics and pushes the result into the gyro.
   *
   * <p>Called once per robot loop rather than from a timer, so the physics and the code that reads
   * it advance together instead of racing.
   */
  public void update() {
    SimulatedArena.getInstance().simulationPeriodic();

    if (pigeonSim != null) {
      var gyro = drivetrain.getGyroSimulation();
      pigeonSim.setRawYaw(gyro.getGyroReading().getMeasure());
      pigeonSim.setAngularVelocityZ(
          DegreesPerSecond.of(gyro.getMeasuredAngularVelocity().in(DegreesPerSecond)));
    }
  }

  /**
   * Returns where the physics says the robot actually is.
   *
   * <p>This is ground truth, not an estimate. Comparing it against the pose estimator is how you
   * tell whether odometry is working; feeding it back into the robot as if it were a measurement
   * would hide exactly the bugs the simulation exists to find.
   *
   * @return the true field pose
   */
  public Pose2d getPose() {
    return drivetrain.getSimulatedDriveTrainPose();
  }

  /**
   * Teleports the simulated robot.
   *
   * @param pose where to place it
   */
  public void resetPose(Pose2d pose) {
    drivetrain.setSimulationWorldPose(pose);
  }
}
