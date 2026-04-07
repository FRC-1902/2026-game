// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.systems.field;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Contains information for location of field element and other useful reference points.
 *
 * <p>All constants are defined relative to the field coordinate system, from the perspective of the
 * blue alliance station.
 */
public class FieldConstants {

  public static final FieldType fieldType = FieldType.WELDED;

  public static final AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  public static final double fieldLength = aprilTagLayout.getFieldLength();
  public static final double fieldWidth = aprilTagLayout.getFieldWidth();

  public enum FieldType {
    WELDED
  }

  /** Hub related constants. */
  public static class Hub {

    public static final double width = Inches.of(47.0).in(Meters);
    public static final double height = Inches.of(72.0).in(Meters);
    public static final double innerWidth = Inches.of(41.7).in(Meters);
    public static final double innerHeight = Inches.of(56.5).in(Meters);

    public static final Translation2d BLUE_HUB_POSITION =
        new Translation2d(Inches.of(182.11).in(Meters), Inches.of(158.84).in(Meters));

    public static final Translation2d RED_HUB_POSITION =
        new Translation2d(Inches.of(469.11).in(Meters), Inches.of(158.84).in(Meters));

    public static final Pose2d HubPoseBlue = new Pose2d(BLUE_HUB_POSITION, Rotation2d.kZero);

    public static Pose2d getHubPose() {
      return AllianceFlipUtil.shouldFlip()
          ? new Pose2d(RED_HUB_POSITION, Rotation2d.kZero)
          : new Pose2d(BLUE_HUB_POSITION, Rotation2d.kZero);
    }

    public static Translation2d getHubTranslation2d() {
      return getHubPose().getTranslation();
    }

    public static final Translation3d topCenterPoint =
        new Translation3d(BLUE_HUB_POSITION.getX(), BLUE_HUB_POSITION.getY(), height);

    public static final Translation3d innerCenterPoint =
        new Translation3d(BLUE_HUB_POSITION.getX(), BLUE_HUB_POSITION.getY(), innerHeight);
  }
}
