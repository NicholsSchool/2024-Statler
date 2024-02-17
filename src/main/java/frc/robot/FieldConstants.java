// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

// TJG

package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import java.util.List;

/**
 * Contains various field dimensions and useful reference points. Dimensions are in meters, and sets
 * of corners start in the lower left moving clockwise.
 *
 * <p>All translations and poses are stored with the origin at the rightmost point on the BLUE
 * ALLIANCE wall. Use the {@link #allianceFlip(Translation2d)} and {@link #allianceFlip(Pose2d)}
 * methods to flip these values based on the current alliance color.
 */
public final class FieldConstants {
  public static final boolean isWPIField = false; // Red alliance

  public static final double fieldLength = Units.inchesToMeters(651.25);
  public static final double fieldWidth =
      Units.inchesToMeters(315.5) + (isWPIField ? Units.inchesToMeters(3.0) : 0.0);
  public static final double tapeWidth = Units.inchesToMeters(2.0);

  public static final Translation2d amplifierTranslation = new Translation2d(1.828, 7.704);

  // TODO: We need to change the loadign zone cords
  // TODO: We need to change the cords for each april tag
  // TODO: Something that I am forgettting
  // Dimensions for loading zone and substations, including the tape
  public static final class LoadingZone {
    // Region dimensions
    public static final double width = Units.inchesToMeters(99.0);
    public static final double innerX = FieldConstants.fieldLength;
    public static final double midX = fieldLength - Units.inchesToMeters(132.25);
    public static final double outerX = fieldLength - Units.inchesToMeters(264.25);
    public static final double leftY = FieldConstants.fieldWidth;
    public static final double midY = leftY - Units.inchesToMeters(50.5);
    public static final double rightY = leftY - width;
    public static final Translation2d[] regionCorners =
        new Translation2d[] {
          new Translation2d(
              midX, rightY), // Start at lower left next to border with opponent community
          new Translation2d(midX, midY),
          new Translation2d(outerX, midY),
          new Translation2d(outerX, leftY),
          new Translation2d(innerX, leftY),
          new Translation2d(innerX, rightY),
        };

    // TODO: change substations to speaker, amp, nad stage
    // Double substation dimensions
    public static final double doubleSubstationLength = Units.inchesToMeters(14.0);
    public static final double doubleSubstationX = innerX - doubleSubstationLength;
    public static final double doubleSubstationShelfZ = Units.inchesToMeters(37.375);
    public static final double doubleSubstationCenterY = fieldWidth - Units.inchesToMeters(49.76);

    // Single substation dimensions
    public static final double singleSubstationWidth = Units.inchesToMeters(22.75);
    public static final double singleSubstationLeftX =
        FieldConstants.fieldLength - doubleSubstationLength - Units.inchesToMeters(88.77);
    public static final double singleSubstationCenterX =
        singleSubstationLeftX + (singleSubstationWidth / 2.0);
    public static final double singleSubstationRightX =
        singleSubstationLeftX + singleSubstationWidth;
    public static final Translation2d singleSubstationTranslation =
        new Translation2d(singleSubstationCenterX, leftY);

    public static final double singleSubstationHeight = Units.inchesToMeters(18.0);
    public static final double singleSubstationLowZ = Units.inchesToMeters(27.125);
    public static final double singleSubstationCenterZ =
        singleSubstationLowZ + (singleSubstationHeight / 2.0);
    public static final double singleSubstationHighZ =
        singleSubstationLowZ + singleSubstationHeight;
  }

  // Locations of staged game pieces
  public static final class StagingLocations {
    public static final double centerOffsetX = Units.inchesToMeters(47.36);
    public static final double positionX = fieldLength / 2.0 - Units.inchesToMeters(47.36);
    public static final double firstY = Units.inchesToMeters(36.19);
    public static final double separationY = Units.inchesToMeters(48.0);
    public static final Translation2d[] translations = new Translation2d[4];

    static {
      for (int i = 0; i < translations.length; i++) {
        translations[i] = new Translation2d(positionX, firstY + (i * separationY));
      }
    }
  }

  public static final AprilTagFieldLayout aprilTags =
      new AprilTagFieldLayout(
          List.of(
              new AprilTag(
                  // TODO: Grid Nodes have to change
                  1,
                  new Pose3d(
                      Units.inchesToMeters(593.68),
                      Units.inchesToMeters(9.68),
                      Units.inchesToMeters(53.38),
                      new Rotation3d(120.0, 0.0, Math.PI))),
              new AprilTag(
                  2,
                  new Pose3d(
                      Units.inchesToMeters(637.21),
                      Units.inchesToMeters(34.79),
                      Units.inchesToMeters(53.38),
                      new Rotation3d(120.0, 0.0, Math.PI))),
              new AprilTag(
                  3,
                  new Pose3d(
                      Units.inchesToMeters(652.73),
                      Units.inchesToMeters(196.17),
                      Units.inchesToMeters(57.13),
                      new Rotation3d(180.0, 0.0, Math.PI))),
              new AprilTag(
                  4,
                  new Pose3d(
                      Units.inchesToMeters(652.73),
                      Units.inchesToMeters(218.42),
                      Units.inchesToMeters(57.13),
                      new Rotation3d(180.0, 0.0, Math.PI))),
              new AprilTag(
                  5,
                  new Pose3d(
                      Units.inchesToMeters(578.77),
                      Units.inchesToMeters(323.00),
                      Units.inchesToMeters(53.38),
                      new Rotation3d(270.0, 0.0, Math.PI))),
              new AprilTag(
                  6,
                  new Pose3d(
                      Units.inchesToMeters(72.5),
                      Units.inchesToMeters(323.00),
                      Units.inchesToMeters(53.38),
                      new Rotation3d(270.0, 0.0, Math.PI))),
              new AprilTag(
                  7,
                  new Pose3d(
                      Units.inchesToMeters(-1.50),
                      Units.inchesToMeters(218.42),
                      Units.inchesToMeters(57.13),
                      new Rotation3d(0.0, 0.0, Math.PI))),
              new AprilTag(
                  8,
                  new Pose3d(
                      Units.inchesToMeters(-1.50),
                      Units.inchesToMeters(196.17),
                      Units.inchesToMeters(57.13),
                      new Rotation3d(0.0, 0.0, Math.PI))),
              new AprilTag(
                  9,
                  new Pose3d(
                      Units.inchesToMeters(14.02),
                      Units.inchesToMeters(34.79),
                      Units.inchesToMeters(53.38),
                      new Rotation3d(60.0, 0.0, Math.PI))),
              new AprilTag(
                  10,
                  new Pose3d(
                      Units.inchesToMeters(57.54),
                      Units.inchesToMeters(9.68),
                      Units.inchesToMeters(53.38),
                      new Rotation3d(60.0, 0.0, Math.PI))),
              new AprilTag(
                  11,
                  new Pose3d(
                      Units.inchesToMeters(468.69),
                      Units.inchesToMeters(146.19),
                      Units.inchesToMeters(52.00),
                      new Rotation3d(300.0, 0.0, Math.PI))),
              new AprilTag(
                  12,
                  new Pose3d(
                      Units.inchesToMeters(468.69),
                      Units.inchesToMeters(177.10),
                      Units.inchesToMeters(52.00),
                      new Rotation3d(60.0, 0.0, Math.PI))),
              new AprilTag(
                  13,
                  new Pose3d(
                      Units.inchesToMeters(441.74),
                      Units.inchesToMeters(161.62),
                      Units.inchesToMeters(52.00),
                      new Rotation3d(180.0, 0.0, Math.PI))),
              new AprilTag(
                  14,
                  new Pose3d(
                      Units.inchesToMeters(209.48),
                      Units.inchesToMeters(161.62),
                      Units.inchesToMeters(52.00),
                      new Rotation3d(0.0, 0.0, Math.PI))),
              new AprilTag(
                  15,
                  new Pose3d(
                      Units.inchesToMeters(182.73),
                      Units.inchesToMeters(177.10),
                      Units.inchesToMeters(52.00),
                      new Rotation3d(120.0, 0.0, Math.PI))),
              new AprilTag(
                  16,
                  new Pose3d(
                      Units.inchesToMeters(182.73),
                      Units.inchesToMeters(146.19),
                      Units.inchesToMeters(52.00),
                      new Rotation3d(240.0, 0.0, Math.PI)))), // TODO:error here no clue why
          fieldLength,
          fieldWidth);
}
