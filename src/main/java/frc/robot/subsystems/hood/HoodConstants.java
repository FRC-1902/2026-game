package frc.robot.subsystems.hood;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/** Constants for the Hood subsystem. */
public class HoodConstants {
  // Motor CAN ID
  public static final int HOOD_MOTOR_ID = 0; // TODO: Set the correct CAN ID

  // Encoder DIO port
  public static final int HOOD_ENCODER_DIO_PORT = 0; // TODO: Set the correct DIO port

  // Mechanical ratios
  // Motor -> 9:1 planetary -> 3:1 planetary -> 24T -> 24T (center axle) -> 48T (hood)
  public static final double MOTOR_TO_CENTER_AXLE_RATIO = 9.0 * 3.0 * (24.0 / 24.0); // 27:1
  public static final double CENTER_AXLE_TO_HOOD_RATIO = 48.0 / 24.0; // 2:1
  public static final double MOTOR_TO_HOOD_RATIO =
      MOTOR_TO_CENTER_AXLE_RATIO * CENTER_AXLE_TO_HOOD_RATIO; // 54:1

  // Encoder ratios
  // Through Bore encoder belted to hood via 48T (hood) -> 24T (encoder)
  // 2 encoder rotations = 1 hood rotation
  public static final double ENCODER_TO_HOOD_RATIO = 2.0; // 2:1

  // PID
  public static final double HOOD_KP = 0.1; // TODO: Tune PID (or basically just P)
  public static final double HOOD_KI = 0.0;
  public static final double HOOD_KD = 0.0;

  // Gravity feedforward constant (percent output at full gravity)
  public static final double HOOD_KCOS = 0.0; // TODO: Tune this value

  // Soft limits (in degrees)
  public static final double HOOD_MIN_ANGLE = 0.0; // TODO: Set based on physical limits
  public static final double HOOD_MAX_ANGLE = 90.0; // TODO: Set based on physical limits

  // Tolerance for reaching target angle (in degrees)
  public static final double HOOD_ANGLE_TOLERANCE = 3.0;

  public static final double HOOD_IZONE = 3.0; // TODO: Tune IZONE vlue

  // Distance to angle interpolation table
  // Distances in meters, angles in degrees
  public static final InterpolatingDoubleTreeMap DISTANCE_TO_ANGLE_MAP =
      new InterpolatingDoubleTreeMap();

  static {
    // TODO: Fill in with real testing data
    // Format: DISTANCE_TO_ANGLE_MAP.put(distance_in_meters, angle_in_degrees);
    DISTANCE_TO_ANGLE_MAP.put(1.0, 30.0);
    DISTANCE_TO_ANGLE_MAP.put(2.0, 35.0);
    DISTANCE_TO_ANGLE_MAP.put(3.0, 40.0);
    DISTANCE_TO_ANGLE_MAP.put(4.0, 45.0);
    DISTANCE_TO_ANGLE_MAP.put(5.0, 50.0);
    DISTANCE_TO_ANGLE_MAP.put(6.0, 55.0);
  }

  // Absolute encoder offset (in degrees)
  // This is the angle reading when the hood is at 0 degrees
  public static final double ENCODER_OFFSET = 0.0; // TODO: Set based on calibration
}
