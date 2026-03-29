// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS =
      new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED = Units.feetToMeters(14.5);

  // Maximum speed of the robot in meters per second, used to limit acceleration.

  //  public static final class AutonConstants
  //  {
  //
  //    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
  //    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
  //  }

  public static final class DrivebaseConstants {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants {

    // Joystick Deadband
    public static final double DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }

  public static final class VisionConstants {

    /**
     * Standard deviations for vision pose estimates when only one AprilTag is visible. Format: [x,
     * y, theta] in meters and radians.
     */
    public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(0.3, 0.3, 0.5);

    /**
     * Standard deviations for vision pose estimates when multiple AprilTags are visible. Format:
     * [x, y, theta] in meters and radians.
     */
    public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.1, 0.1, 0.2);

    /** Camera configurations for all cameras on the robot. */
    public static final class Cameras {

      // TODO: confirm front camera translations and rotations
      // Front Left Camera
      public static final String FRONT_LEFT_NAME = "arducamThree";
      public static final Transform3d FRONT_LEFT_ROBOT_TO_CAM =
          new Transform3d(
              new Translation3d(
                  Units.inchesToMeters(-8.722503),
                  Units.inchesToMeters(12.076808),
                  Units.inchesToMeters(18.970713)),
              new Rotation3d(Math.toRadians(-10), Math.toRadians(-22.67), Math.toRadians(25.696)));

      // Front Right Camera
      public static final String FRONT_RIGHT_NAME = "ArducamZero";
      public static final Transform3d FRONT_RIGHT_ROBOT_TO_CAM =
          new Transform3d(
              new Translation3d(
                  Units.inchesToMeters(-4.697477),
                  Units.inchesToMeters(-10.859125),
                  Units.inchesToMeters(18.936709)),
              new Rotation3d(
                  Math.toRadians(10), Math.toRadians(-19.4175), Math.toRadians(-18.195)));

      // Back Left Camera
      // XXX: confirm pitch values from cad, this was determined experimentally unlike the rest
      public static final String BACK_LEFT_NAME = "ArducamTwo";
      public static final Transform3d BACK_LEFT_ROBOT_TO_CAM =
          new Transform3d(
              new Translation3d(
                  Units.inchesToMeters(-13.635),
                  Units.inchesToMeters(2.078),
                  Units.inchesToMeters(6.725)),
              new Rotation3d(Math.toRadians(2.95), Math.toRadians(-11.5), Math.toRadians(155.917)));

      // Back Right Camera
      // XXX: confirm pitch values from cad, this was determined experimentally unlike the rest
      public static final String BACK_RIGHT_NAME = "arducamOne";
      public static final Transform3d BACK_RIGHT_ROBOT_TO_CAM =
          new Transform3d(
              new Translation3d(
                  Units.inchesToMeters(-13.620134),
                  Units.inchesToMeters(-1.827022),
                  Units.inchesToMeters(6.862401)),
              new Rotation3d(
                  Math.toRadians(-2.95), Math.toRadians(-11.5), Math.toRadians(204.083)));
    }
  }
}
