// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.utils;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Measure;
import edu.wpi.first.util.struct.Struct;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

/**
 * One place to send a value, with a choice of where it ends up.
 *
 * <p>The robot has two telemetry systems that never met. Phoenix's {@link SignalLogger} writes hoot
 * files to disk, so anything logged through it is invisible while driving. NetworkTables is live
 * but keeps nothing, so anything sent to the dashboard is gone the moment the robot disables. The
 * result is that what gets recorded is not what you can watch, and what you can watch is not
 * recorded.
 *
 * <p>Every method here takes a {@link Destination} saying which of those you want, so a single call
 * can do both under one path name. That last part matters: correlating a live observation with the
 * recording of it only works if they agree on what the value is called.
 *
 * <h2>Paths</h2>
 *
 * <p>Always start with a slash and use PascalCase segments, no spaces: {@code
 * /Swerve/Chassis/Pose}, {@code /Vision/limelight-up/Pose}, {@code
 * /Controllers/XboxController-0/IsConnected}.
 *
 * <h2>Units</h2>
 *
 * <p>Prefer {@link #logValue(String, Measure)}, which carries the unit into the log for you, over
 * logging a bare double. Where a raw double is unavoidable, pass the unit string rather than the
 * empty string that used to be hard coded here.
 *
 * <p>Phoenix keeps logging every CTRE device signal automatically for as long as {@link
 * #startLogging()} has been called. Nothing in this class affects that; it only decides where the
 * values <i>your own code</i> computes are written.
 */
public final class Logging {
  /** Where a logged value should go. */
  public enum Destination {
    /** Written to the log file only. Use for anything high rate you review afterwards. */
    LOG_ONLY,

    /** Published to NetworkTables only. Use for tuning values not worth keeping. */
    DASHBOARD_ONLY,

    /** Written to the log and published live. Use for anything you watch while driving. */
    BOTH
  }

  private static final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();

  // Publishers are cached because creating one per call would leak a NetworkTables entry every
  // loop. Concurrent because logging happens from whatever thread the caller is on.
  private static final Map<String, StructPublisher<?>> structPublishers = new ConcurrentHashMap<>();
  private static final Map<String, StructArrayPublisher<?>> structArrayPublishers =
      new ConcurrentHashMap<>();
  private static final Map<String, DoublePublisher> doublePublishers = new ConcurrentHashMap<>();
  private static final Map<String, BooleanPublisher> booleanPublishers = new ConcurrentHashMap<>();
  private static final Map<String, StringPublisher> stringPublishers = new ConcurrentHashMap<>();

  private static volatile Destination defaultDestination = Destination.LOG_ONLY;

  private Logging() {}

  /**
   * Sets the destination used by calls that do not name one.
   *
   * <p>Defaults to {@link Destination#LOG_ONLY}, so an unmarked call is cheap and dashboard traffic
   * is something you opt into. Flipping this to {@link Destination#BOTH} turns the whole robot
   * chatty for a debugging session without touching any call site.
   *
   * @param destination the destination to use when none is given
   */
  public static void setDefaultDestination(Destination destination) {
    defaultDestination = destination;
  }

  /**
   * Returns the destination used by calls that do not name one.
   *
   * @return the current default destination
   */
  public static Destination getDefaultDestination() {
    return defaultDestination;
  }

  /** Starts the signal logger, which also begins automatic capture of every CTRE device signal. */
  public static void startLogging() {
    SignalLogger.start();
  }

  /** Stops the signal logger. */
  public static void stopLogging() {
    SignalLogger.stop();
  }

  /* ------------------------------------------------------------------ structs */

  /**
   * Logs any type WPILib can serialise as a struct.
   *
   * <p>This is the escape hatch behind the typed helpers below. Logging a struct rather than
   * flattening it into an array of doubles is what lets AdvantageScope treat a pose as a pose
   * instead of three anonymous numbers.
   *
   * @param <T> the logged type
   * @param path the path to log under
   * @param struct the struct serialiser, for example {@code Pose2d.struct}
   * @param value the value to log
   * @param destination where the value should go
   */
  public static <T> void logStruct(
      String path, Struct<T> struct, T value, Destination destination) {
    if (writesToLog(destination)) {
      SignalLogger.writeStruct(path, struct, value);
    }
    if (writesToDashboard(destination)) {
      structPublisher(path, struct).set(value);
    }
  }

  /**
   * Logs any type WPILib can serialise as a struct, using the default destination.
   *
   * @param <T> the logged type
   * @param path the path to log under
   * @param struct the struct serialiser
   * @param value the value to log
   */
  public static <T> void logStruct(String path, Struct<T> struct, T value) {
    logStruct(path, struct, value, defaultDestination);
  }

  /**
   * Logs a struct sample that describes a moment in the past.
   *
   * <p>Only the log carries the age; NetworkTables has no notion of a stale sample, so the
   * dashboard sees it as current.
   *
   * @param <T> the logged type
   * @param path the path to log under
   * @param struct the struct serialiser
   * @param value the value to log
   * @param latencySeconds how old the sample is, in seconds
   * @param destination where the value should go
   */
  public static <T> void logStruct(
      String path, Struct<T> struct, T value, double latencySeconds, Destination destination) {
    if (writesToLog(destination)) {
      SignalLogger.writeStruct(path, struct, value, latencySeconds);
    }
    if (writesToDashboard(destination)) {
      structPublisher(path, struct).set(value);
    }
  }

  /**
   * Logs an array of struct serialisable values.
   *
   * @param <T> the logged type
   * @param path the path to log under
   * @param struct the struct serialiser
   * @param values the values to log
   * @param destination where the values should go
   */
  public static <T> void logStructArray(
      String path, Struct<T> struct, T[] values, Destination destination) {
    if (writesToLog(destination)) {
      SignalLogger.writeStructArray(path, struct, values);
    }
    if (writesToDashboard(destination)) {
      structArrayPublisher(path, struct).set(values);
    }
  }

  /**
   * Logs an array of struct serialisable values, using the default destination.
   *
   * @param <T> the logged type
   * @param path the path to log under
   * @param struct the struct serialiser
   * @param values the values to log
   */
  public static <T> void logStructArray(String path, Struct<T> struct, T[] values) {
    logStructArray(path, struct, values, defaultDestination);
  }

  /* ------------------------------------------------------------------- poses */

  /**
   * Logs a pose.
   *
   * @param path the path to log under
   * @param pose the pose to log
   * @param destination where the pose should go
   */
  public static void logPose(String path, Pose2d pose, Destination destination) {
    logStruct(path, Pose2d.struct, pose, destination);
  }

  /**
   * Logs a pose, using the default destination.
   *
   * @param path the path to log under
   * @param pose the pose to log
   */
  public static void logPose(String path, Pose2d pose) {
    logStruct(path, Pose2d.struct, pose, defaultDestination);
  }

  /**
   * Logs a pose that describes a moment in the past, such as a vision estimate.
   *
   * @param path the path to log under
   * @param pose the pose to log
   * @param latencySeconds how old the sample is, in seconds
   * @param destination where the pose should go
   */
  public static void logPose(
      String path, Pose2d pose, double latencySeconds, Destination destination) {
    logStruct(path, Pose2d.struct, pose, latencySeconds, destination);
  }

  /**
   * Logs a three dimensional pose.
   *
   * @param path the path to log under
   * @param pose the pose to log
   * @param destination where the pose should go
   */
  public static void logPose(String path, Pose3d pose, Destination destination) {
    logStruct(path, Pose3d.struct, pose, destination);
  }

  /**
   * Logs a three dimensional pose, using the default destination.
   *
   * @param path the path to log under
   * @param pose the pose to log
   */
  public static void logPose(String path, Pose3d pose) {
    logStruct(path, Pose3d.struct, pose, defaultDestination);
  }

  /**
   * Logs a list of poses, such as the tags a camera can currently see.
   *
   * @param path the path to log under
   * @param poses the poses to log
   * @param destination where the poses should go
   */
  public static void logPoses(String path, Pose2d[] poses, Destination destination) {
    logStructArray(path, Pose2d.struct, poses, destination);
  }

  /**
   * Logs a list of three dimensional poses.
   *
   * @param path the path to log under
   * @param poses the poses to log
   * @param destination where the poses should go
   */
  public static void logPoses(String path, Pose3d[] poses, Destination destination) {
    logStructArray(path, Pose3d.struct, poses, destination);
  }

  /**
   * Logs a rotation.
   *
   * @param path the path to log under
   * @param rotation the rotation to log
   * @param destination where the rotation should go
   */
  public static void logRotation2d(String path, Rotation2d rotation, Destination destination) {
    logStruct(path, Rotation2d.struct, rotation, destination);
  }

  /* ------------------------------------------------------------------ swerve */

  /**
   * Logs a set of chassis speeds.
   *
   * @param path the path to log under
   * @param speeds the speeds to log
   * @param destination where the speeds should go
   */
  public static void logChassisSpeeds(String path, ChassisSpeeds speeds, Destination destination) {
    logStruct(path, ChassisSpeeds.struct, speeds, destination);
  }

  /**
   * Logs the state of every swerve module.
   *
   * @param path the path to log under
   * @param states the module states to log
   * @param destination where the states should go
   */
  public static void logModuleStates(
      String path, SwerveModuleState[] states, Destination destination) {
    logStructArray(path, SwerveModuleState.struct, states, destination);
  }

  /**
   * Logs the position of every swerve module.
   *
   * @param path the path to log under
   * @param positions the module positions to log
   * @param destination where the positions should go
   */
  public static void logModulePositions(
      String path, SwerveModulePosition[] positions, Destination destination) {
    logStructArray(path, SwerveModulePosition.struct, positions, destination);
  }

  /* ----------------------------------------------------------------- scalars */

  /**
   * Logs a measure, carrying its unit into the log.
   *
   * <p>Prefer this over {@link #logDouble(String, double, String, Destination)}: passing the
   * measure itself means the unit cannot drift out of step with the number. The dashboard receives
   * the value in its base unit, since NetworkTables has no unit metadata.
   *
   * @param path the path to log under
   * @param value the measure to log
   * @param destination where the value should go
   */
  public static void logValue(String path, Measure<?> value, Destination destination) {
    if (writesToLog(destination)) {
      SignalLogger.writeValue(path, value);
    }
    if (writesToDashboard(destination)) {
      doublePublisher(path).set(value.baseUnitMagnitude());
    }
  }

  /**
   * Logs a measure, using the default destination.
   *
   * @param path the path to log under
   * @param value the measure to log
   */
  public static void logValue(String path, Measure<?> value) {
    logValue(path, value, defaultDestination);
  }

  /**
   * Logs a number together with its unit.
   *
   * @param path the path to log under
   * @param value the value to log
   * @param unit the unit the value is in, for example {@code "m/s"} or {@code "volts"}
   * @param destination where the value should go
   */
  public static void logDouble(String path, double value, String unit, Destination destination) {
    if (writesToLog(destination)) {
      SignalLogger.writeDouble(path, value, unit);
    }
    if (writesToDashboard(destination)) {
      doublePublisher(path).set(value);
    }
  }

  /**
   * Logs a number together with its unit, using the default destination.
   *
   * @param path the path to log under
   * @param value the value to log
   * @param unit the unit the value is in
   */
  public static void logDouble(String path, double value, String unit) {
    logDouble(path, value, unit, defaultDestination);
  }

  /**
   * Logs a boolean.
   *
   * @param path the path to log under
   * @param value the value to log
   * @param destination where the value should go
   */
  public static void logBoolean(String path, boolean value, Destination destination) {
    if (writesToLog(destination)) {
      SignalLogger.writeBoolean(path, value);
    }
    if (writesToDashboard(destination)) {
      booleanPublisher(path).set(value);
    }
  }

  /**
   * Logs a boolean, using the default destination.
   *
   * @param path the path to log under
   * @param value the value to log
   */
  public static void logBoolean(String path, boolean value) {
    logBoolean(path, value, defaultDestination);
  }

  /**
   * Logs a string.
   *
   * @param path the path to log under
   * @param value the value to log
   * @param destination where the value should go
   */
  public static void logString(String path, String value, Destination destination) {
    if (writesToLog(destination)) {
      SignalLogger.writeString(path, value);
    }
    if (writesToDashboard(destination)) {
      stringPublisher(path).set(value);
    }
  }

  /**
   * Logs a string, using the default destination.
   *
   * @param path the path to log under
   * @param value the value to log
   */
  public static void logString(String path, String value) {
    logString(path, value, defaultDestination);
  }

  /* ---------------------------------------------------------------- internals */

  // Package private rather than private so the routing decision can be asserted directly. Testing
  // it through the public methods would mean calling SignalLogger, which needs the Phoenix natives
  // on java.library.path, and those are not worth pulling into the test runtime for two branches.
  static boolean writesToLog(Destination destination) {
    return destination != Destination.DASHBOARD_ONLY;
  }

  static boolean writesToDashboard(Destination destination) {
    return destination != Destination.LOG_ONLY;
  }

  // The topic getters below take the path exactly as given, unlike getTable, which prepends a
  // leading slash for you. Paths here are documented to start with one, so the topics land where
  // the name says rather than beside the root.
  @SuppressWarnings("unchecked")
  private static <T> StructPublisher<T> structPublisher(String path, Struct<T> struct) {
    return (StructPublisher<T>)
        structPublishers.computeIfAbsent(
            path, key -> ntInstance.getStructTopic(key, struct).publish());
  }

  @SuppressWarnings("unchecked")
  private static <T> StructArrayPublisher<T> structArrayPublisher(String path, Struct<T> struct) {
    return (StructArrayPublisher<T>)
        structArrayPublishers.computeIfAbsent(
            path, key -> ntInstance.getStructArrayTopic(key, struct).publish());
  }

  private static DoublePublisher doublePublisher(String path) {
    return doublePublishers.computeIfAbsent(path, key -> ntInstance.getDoubleTopic(key).publish());
  }

  private static BooleanPublisher booleanPublisher(String path) {
    return booleanPublishers.computeIfAbsent(
        path, key -> ntInstance.getBooleanTopic(key).publish());
  }

  private static StringPublisher stringPublisher(String path) {
    return stringPublishers.computeIfAbsent(path, key -> ntInstance.getStringTopic(key).publish());
  }
}
