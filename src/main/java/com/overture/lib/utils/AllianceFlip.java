// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.overture.lib.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Moves field coordinates from one alliance to the other, and from one side of the field to the
 * other.
 *
 * <p>Robot code is written once, in blue alliance coordinates: the origin in the corner to the
 * right of the blue drivers, x growing towards the red wall and y growing to their left. {@code
 * flip} takes a position, heading, pose or field relative speed written that way and returns the
 * one that means the same thing for the red alliance, still measured from the blue origin.
 *
 * <p>This is the arithmetic PathPlanner's and BLine's FlippingUtil do, with the same field size, so
 * a target flipped here lands exactly where a path flipped by either of them does. It lives in the
 * library so that code which only aims at something does not depend on which path follower the
 * robot happens to use.
 */
public final class AllianceFlip {
  /** How the red half of the field is laid out relative to the blue half. */
  public enum FieldSymmetry {
    /**
     * The red half is the blue half turned 180 degrees about the middle of the field, so what is on
     * the blue drivers' left is on the red drivers' left too. 2022, 2025 and 2026 are like this.
     */
    ROTATIONAL,

    /**
     * The red half is the blue half seen in a mirror standing across the middle of the field, so
     * what is on the blue drivers' left is on the red drivers' right. 2023 and 2024 are like this.
     */
    MIRRORED
  }

  // The 2026 field. The size is the one PathPlanner and BLine use, on purpose: WPILib's AprilTag
  // layout says 16.541 by 8.069, and a millimetre is not worth flipping a target to a slightly
  // different place than the path that drives to it.
  private static FieldSymmetry symmetry = FieldSymmetry.ROTATIONAL;
  private static double fieldLength = 16.54;
  private static double fieldWidth = 8.07;

  private AllianceFlip() {}

  /**
   * Describes a field other than this season's.
   *
   * <p>Only needed to run on an older field, at an off season event for example. Call it first
   * thing in the Robot constructor: anything that has already read the field size, a constant
   * computed from {@link #getFieldWidth()} say, keeps the number it read.
   *
   * @param symmetry how the red half relates to the blue half
   * @param fieldLengthMeters the size of the field along x, from one alliance wall to the other
   * @param fieldWidthMeters the size of the field along y
   */
  public static void configure(
      FieldSymmetry symmetry, double fieldLengthMeters, double fieldWidthMeters) {
    AllianceFlip.symmetry = symmetry;
    fieldLength = fieldLengthMeters;
    fieldWidth = fieldWidthMeters;
  }

  /**
   * Returns how the red half of the field relates to the blue half.
   *
   * @return the field symmetry
   */
  public static FieldSymmetry getSymmetry() {
    return symmetry;
  }

  /**
   * Returns the size of the field along x, from one alliance wall to the other.
   *
   * @return the field length, in meters
   */
  public static double getFieldLength() {
    return fieldLength;
  }

  /**
   * Returns the size of the field along y.
   *
   * @return the field width, in meters
   */
  public static double getFieldWidth() {
    return fieldWidth;
  }

  /**
   * Flips a position to the other alliance.
   *
   * @param position the position, in blue alliance coordinates
   * @return the matching position on the other alliance's side
   */
  public static Translation2d flip(Translation2d position) {
    return switch (symmetry) {
      case ROTATIONAL -> new Translation2d(
          fieldLength - position.getX(), fieldWidth - position.getY());
      case MIRRORED -> new Translation2d(fieldLength - position.getX(), position.getY());
    };
  }

  /**
   * Flips a heading to the other alliance.
   *
   * @param rotation the heading, in blue alliance coordinates
   * @return the matching heading for the other alliance
   */
  public static Rotation2d flip(Rotation2d rotation) {
    return switch (symmetry) {
      case ROTATIONAL -> rotation.minus(Rotation2d.kPi);
      case MIRRORED -> Rotation2d.kPi.minus(rotation);
    };
  }

  /**
   * Flips a pose to the other alliance.
   *
   * @param pose the pose, in blue alliance coordinates
   * @return the matching pose on the other alliance's side
   */
  public static Pose2d flip(Pose2d pose) {
    return new Pose2d(flip(pose.getTranslation()), flip(pose.getRotation()));
  }

  /**
   * Flips field relative speeds to the other alliance.
   *
   * <p>Field relative only. Robot relative speeds mean the same thing on both alliances and must
   * not be flipped.
   *
   * @param fieldSpeeds the field relative speeds, in blue alliance coordinates
   * @return the matching speeds for the other alliance
   */
  public static ChassisSpeeds flip(ChassisSpeeds fieldSpeeds) {
    return switch (symmetry) {
      case ROTATIONAL -> new ChassisSpeeds(
          -fieldSpeeds.vxMetersPerSecond,
          -fieldSpeeds.vyMetersPerSecond,
          fieldSpeeds.omegaRadiansPerSecond);
      case MIRRORED -> new ChassisSpeeds(
          -fieldSpeeds.vxMetersPerSecond,
          fieldSpeeds.vyMetersPerSecond,
          -fieldSpeeds.omegaRadiansPerSecond);
    };
  }

  /**
   * Returns a blue alliance position as the alliance we are on needs it: flipped if we are red,
   * untouched if we are blue.
   *
   * <p>Call this where the value is used, not once while the robot is starting. The alliance is not
   * known until the driver station connects, and until then this answers for blue.
   *
   * @param bluePosition the position, in blue alliance coordinates
   * @return the position for our alliance
   */
  public static Translation2d flipIfRed(Translation2d bluePosition) {
    return UtilityFunctions.isRedAlliance() ? flip(bluePosition) : bluePosition;
  }

  /**
   * Returns a blue alliance heading as the alliance we are on needs it. See {@link
   * #flipIfRed(Translation2d)} for when to call it.
   *
   * @param blueRotation the heading, in blue alliance coordinates
   * @return the heading for our alliance
   */
  public static Rotation2d flipIfRed(Rotation2d blueRotation) {
    return UtilityFunctions.isRedAlliance() ? flip(blueRotation) : blueRotation;
  }

  /**
   * Returns a blue alliance pose as the alliance we are on needs it. See {@link
   * #flipIfRed(Translation2d)} for when to call it.
   *
   * @param bluePose the pose, in blue alliance coordinates
   * @return the pose for our alliance
   */
  public static Pose2d flipIfRed(Pose2d bluePose) {
    return UtilityFunctions.isRedAlliance() ? flip(bluePose) : bluePose;
  }

  /**
   * Returns blue alliance field relative speeds as the alliance we are on needs them. See {@link
   * #flipIfRed(Translation2d)} for when to call it.
   *
   * @param blueFieldSpeeds the field relative speeds, in blue alliance coordinates
   * @return the speeds for our alliance
   */
  public static ChassisSpeeds flipIfRed(ChassisSpeeds blueFieldSpeeds) {
    return UtilityFunctions.isRedAlliance() ? flip(blueFieldSpeeds) : blueFieldSpeeds;
  }

  /**
   * Moves a position to the other side of the field, left to right, staying on the same alliance.
   *
   * <p>Not an alliance flip, and not related to {@link FieldSymmetry#MIRRORED}. It is for writing
   * one of two matching things, a left and a right pass target say, in terms of the other.
   *
   * @param position the position
   * @return the position the same distance from the opposite side wall
   */
  public static Translation2d mirrorLeftRight(Translation2d position) {
    return new Translation2d(position.getX(), fieldWidth - position.getY());
  }

  /**
   * Turns a heading into the one that matches it on the other side of the field, left to right.
   *
   * @param rotation the heading
   * @return the mirrored heading
   */
  public static Rotation2d mirrorLeftRight(Rotation2d rotation) {
    return rotation.unaryMinus();
  }

  /**
   * Moves a pose to the other side of the field, left to right, staying on the same alliance.
   *
   * @param pose the pose
   * @return the mirrored pose
   */
  public static Pose2d mirrorLeftRight(Pose2d pose) {
    return new Pose2d(mirrorLeftRight(pose.getTranslation()), mirrorLeftRight(pose.getRotation()));
  }
}
