// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;

import com.overture.lib.utils.AllianceFlip.FieldSymmetry;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Checks both field symmetries, the left to right mirror and the flip that follows the alliance.
 */
class AllianceFlipTest {
  private static final double kEpsilon = 1e-9;

  // Read before any test reconfigures the field, so they are what the library ships with.
  private static final FieldSymmetry kShippedSymmetry = AllianceFlip.getSymmetry();
  private static final double kShippedLength = AllianceFlip.getFieldLength();
  private static final double kShippedWidth = AllianceFlip.getFieldWidth();

  @BeforeAll
  static void setUp() {
    HAL.initialize(500, 0);
  }

  // The field and the alliance are both global, so every test hands them back as it found them.
  @AfterEach
  void restore() {
    AllianceFlip.configure(kShippedSymmetry, kShippedLength, kShippedWidth);
    setAlliance(AllianceStationID.Unknown);
  }

  private static void setAlliance(AllianceStationID station) {
    DriverStationSim.setAllianceStationId(station);
    DriverStationSim.notifyNewData();
  }

  private static void assertTranslation(double x, double y, Translation2d actual) {
    assertEquals(x, actual.getX(), kEpsilon, "x");
    assertEquals(y, actual.getY(), kEpsilon, "y");
  }

  private static void assertDegrees(double degrees, Rotation2d actual) {
    // Compared as an angle, so 180 and -180 count as the same heading.
    assertEquals(0.0, actual.minus(Rotation2d.fromDegrees(degrees)).getDegrees(), 1e-6, "heading");
  }

  private static void assertSpeeds(double vx, double vy, double omega, ChassisSpeeds actual) {
    assertEquals(vx, actual.vxMetersPerSecond, kEpsilon, "vx");
    assertEquals(vy, actual.vyMetersPerSecond, kEpsilon, "vy");
    assertEquals(omega, actual.omegaRadiansPerSecond, kEpsilon, "omega");
  }

  @Test
  void shipsWithThe2026Field() {
    assertSame(FieldSymmetry.ROTATIONAL, kShippedSymmetry);
    assertEquals(16.54, kShippedLength, kEpsilon);
    assertEquals(8.07, kShippedWidth, kEpsilon);
  }

  @Test
  void rotationalFlipTurnsEverythingAboutTheFieldCenter() {
    AllianceFlip.configure(FieldSymmetry.ROTATIONAL, 16.54, 8.07);

    assertTranslation(12.54, 5.57, AllianceFlip.flip(new Translation2d(4.0, 2.5)));
    assertDegrees(-150.0, AllianceFlip.flip(Rotation2d.fromDegrees(30.0)));
    assertSpeeds(-1.0, -2.0, 3.0, AllianceFlip.flip(new ChassisSpeeds(1.0, 2.0, 3.0)));

    Pose2d flipped = AllianceFlip.flip(new Pose2d(4.0, 2.5, Rotation2d.fromDegrees(30.0)));
    assertTranslation(12.54, 5.57, flipped.getTranslation());
    assertDegrees(-150.0, flipped.getRotation());

    // The one point a half turn leaves where it was.
    assertTranslation(8.27, 4.035, AllianceFlip.flip(new Translation2d(8.27, 4.035)));
  }

  @Test
  void mirroredFlipKeepsTheSideOfTheField() {
    AllianceFlip.configure(FieldSymmetry.MIRRORED, 16.54, 8.07);

    // y survives: a mirror across the middle swaps the alliances and nothing else.
    assertTranslation(12.54, 2.5, AllianceFlip.flip(new Translation2d(4.0, 2.5)));
    assertDegrees(150.0, AllianceFlip.flip(Rotation2d.fromDegrees(30.0)));
    assertSpeeds(-1.0, 2.0, -3.0, AllianceFlip.flip(new ChassisSpeeds(1.0, 2.0, 3.0)));

    Pose2d flipped = AllianceFlip.flip(new Pose2d(4.0, 2.5, Rotation2d.fromDegrees(30.0)));
    assertTranslation(12.54, 2.5, flipped.getTranslation());
    assertDegrees(150.0, flipped.getRotation());
  }

  @Test
  void flippingTwiceComesBackUnderBothSymmetries() {
    Pose2d pose = new Pose2d(3.2, 6.9, Rotation2d.fromDegrees(-73.0));
    ChassisSpeeds speeds = new ChassisSpeeds(1.5, -0.4, 2.2);

    for (FieldSymmetry symmetry : FieldSymmetry.values()) {
      AllianceFlip.configure(symmetry, 16.54, 8.07);

      Pose2d back = AllianceFlip.flip(AllianceFlip.flip(pose));
      assertTranslation(3.2, 6.9, back.getTranslation());
      assertDegrees(-73.0, back.getRotation());
      assertSpeeds(1.5, -0.4, 2.2, AllianceFlip.flip(AllianceFlip.flip(speeds)));
    }
  }

  @Test
  void usesTheConfiguredFieldSize() {
    // The 2024 field: mirrored, and a different size.
    AllianceFlip.configure(FieldSymmetry.MIRRORED, 16.541, 8.211);

    assertEquals(16.541, AllianceFlip.getFieldLength(), kEpsilon);
    assertEquals(8.211, AllianceFlip.getFieldWidth(), kEpsilon);
    assertTranslation(15.541, 1.0, AllianceFlip.flip(new Translation2d(1.0, 1.0)));
    assertTranslation(1.0, 7.211, AllianceFlip.mirrorLeftRight(new Translation2d(1.0, 1.0)));
  }

  @Test
  void mirrorLeftRightSwapsSidesAndIgnoresTheSymmetry() {
    for (FieldSymmetry symmetry : FieldSymmetry.values()) {
      AllianceFlip.configure(symmetry, 16.54, 8.07);

      // x survives: the robot stays on its own alliance's half.
      assertTranslation(4.0, 5.57, AllianceFlip.mirrorLeftRight(new Translation2d(4.0, 2.5)));
      assertDegrees(-30.0, AllianceFlip.mirrorLeftRight(Rotation2d.fromDegrees(30.0)));

      Pose2d mirrored =
          AllianceFlip.mirrorLeftRight(new Pose2d(4.0, 2.5, Rotation2d.fromDegrees(30.0)));
      assertTranslation(4.0, 5.57, mirrored.getTranslation());
      assertDegrees(-30.0, mirrored.getRotation());
    }
  }

  @Test
  void flipIfRedLeavesBlueAndUnknownAlone() {
    Translation2d position = new Translation2d(4.0, 2.5);
    Rotation2d rotation = Rotation2d.fromDegrees(30.0);
    Pose2d pose = new Pose2d(position, rotation);
    ChassisSpeeds speeds = new ChassisSpeeds(1.0, 2.0, 3.0);

    for (AllianceStationID station :
        new AllianceStationID[] {AllianceStationID.Blue2, AllianceStationID.Unknown}) {
      setAlliance(station);

      assertSame(position, AllianceFlip.flipIfRed(position));
      assertSame(rotation, AllianceFlip.flipIfRed(rotation));
      assertSame(pose, AllianceFlip.flipIfRed(pose));
      assertSame(speeds, AllianceFlip.flipIfRed(speeds));
    }
  }

  @Test
  void flipIfRedFlipsOnRed() {
    setAlliance(AllianceStationID.Red2);

    assertTranslation(12.54, 5.57, AllianceFlip.flipIfRed(new Translation2d(4.0, 2.5)));
    assertDegrees(-150.0, AllianceFlip.flipIfRed(Rotation2d.fromDegrees(30.0)));
    assertSpeeds(-1.0, -2.0, 3.0, AllianceFlip.flipIfRed(new ChassisSpeeds(1.0, 2.0, 3.0)));

    Pose2d flipped = AllianceFlip.flipIfRed(new Pose2d(4.0, 2.5, Rotation2d.fromDegrees(30.0)));
    assertTranslation(12.54, 5.57, flipped.getTranslation());
    assertDegrees(-150.0, flipped.getRotation());
  }
}
