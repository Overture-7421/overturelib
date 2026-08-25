// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.utils;

import static edu.wpi.first.units.Units.Meters;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.overture.lib.utils.Logging.Destination;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructSubscriber;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Covers the routing decision, which is the whole point of the facade.
 *
 * <p>Anything that writes to the log calls into Phoenix, and the Phoenix natives are deliberately
 * absent from the test runtime, so these tests exercise the two halves separately: the routing
 * predicates directly, and the NetworkTables side end to end through {@link
 * Destination#DASHBOARD_ONLY}. What is not covered is the one line that hands a value to
 * SignalLogger.
 */
class LoggingTest {
  private static final Pose2d kPose = new Pose2d(1.5, 2.5, Rotation2d.fromDegrees(30));

  @BeforeAll
  static void setUp() {
    HAL.initialize(500, 0);
  }

  @AfterEach
  void restoreDefault() {
    Logging.setDefaultDestination(Destination.LOG_ONLY);
  }

  private static StructSubscriber<Pose2d> watch(String path) {
    return NetworkTableInstance.getDefault()
        .getStructTopic(path, Pose2d.struct)
        .subscribe(Pose2d.kZero);
  }

  @Test
  void routingSendsEachDestinationToTheRightPlaces() {
    // Getting either of these backwards would flood the dashboard or silently drop telemetry
    // somebody is watching, and neither failure announces itself.
    assertTrue(Logging.writesToLog(Destination.LOG_ONLY));
    assertFalse(Logging.writesToDashboard(Destination.LOG_ONLY));

    assertFalse(Logging.writesToLog(Destination.DASHBOARD_ONLY));
    assertTrue(Logging.writesToDashboard(Destination.DASHBOARD_ONLY));

    assertTrue(Logging.writesToLog(Destination.BOTH));
    assertTrue(Logging.writesToDashboard(Destination.BOTH));
  }

  @Test
  void dashboardOnlyPublishesTheValue() {
    String path = "/TestLogging/DashOnly/Pose";
    StructSubscriber<Pose2d> watcher = watch(path);

    Logging.logPose(path, kPose, Destination.DASHBOARD_ONLY);

    assertEquals(kPose, watcher.get());
    assertTrue(watcher.getTopic().exists());
  }

  @Test
  void defaultIsLogOnlySoUnmarkedCallsCostNoBandwidth() {
    assertEquals(Destination.LOG_ONLY, Logging.getDefaultDestination());
  }

  @Test
  void changingTheDefaultChangesWhereUnmarkedCallsGo() {
    String path = "/TestLogging/Default/Pose";
    StructSubscriber<Pose2d> watcher = watch(path);

    Logging.setDefaultDestination(Destination.DASHBOARD_ONLY);
    Logging.logPose(path, kPose);

    assertEquals(kPose, watcher.get(), "flipping the default should not need a call site change");
  }

  @Test
  void pathIsUsedVerbatimIncludingTheLeadingSlash() {
    // getStructTopic does not normalise the way getTable does. A path that lost its leading slash
    // would land beside the root instead of inside it, which is how an earlier pose publisher
    // ended up invisible to anything browsing the table it claimed to be in.
    String path = "/TestLogging/Verbatim/Pose";
    Logging.logPose(path, kPose, Destination.DASHBOARD_ONLY);

    assertTrue(
        NetworkTableInstance.getDefault().getStructTopic(path, Pose2d.struct).exists(),
        "topic should exist at exactly the path given");
  }

  @Test
  void measuresPublishInBaseUnits() {
    String path = "/TestLogging/Units/Distance";
    var watcher = NetworkTableInstance.getDefault().getDoubleTopic(path).subscribe(0.0);

    Logging.logValue(path, Meters.of(2.5), Destination.DASHBOARD_ONLY);

    assertEquals(2.5, watcher.get(), 1e-9);
  }

  @Test
  void repeatedLoggingKeepsPublishingTheLatestValue() {
    // Publishers are cached per path; a cache that handed back a stale or closed publisher would
    // leave the dashboard frozen on the first value rather than failing outright.
    String path = "/TestLogging/Repeat/Pose";
    StructSubscriber<Pose2d> watcher = watch(path);

    for (int i = 0; i < 50; i++) {
      Logging.logPose(path, new Pose2d(i, 0, Rotation2d.kZero), Destination.DASHBOARD_ONLY);
    }

    assertEquals(49.0, watcher.get().getX(), 1e-9);
  }
}
