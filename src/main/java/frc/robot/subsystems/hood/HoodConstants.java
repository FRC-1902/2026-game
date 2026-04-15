package frc.robot.subsystems.hood;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.util.NavigableMap;
import java.util.TreeMap;

/** Constants for the Hood subsystem. */
public class HoodConstants {
  // Motor CAN ID
  public static final int HOOD_MOTOR_ID = 40;

  // Encoder DIO port
  public static final int HOOD_ENCODER_DIO_PORT = 0;

  // Encoder ratios
  // Through Bore encoder belted to hood via 48T (hood) -> 24T (encoder)
  // 2 encoder rotations = 1 hood rotation
  public static final double ENCODER_TO_HOOD_RATIO = 0.5; // 2:1

  // PID
  public static final double HOOD_KP = 0.01;
  public static final double HOOD_KI = 0.0;
  public static final double HOOD_KD = 0.0;

  // Gravity feedforward constant (percent output at full gravity)
  public static final double HOOD_KCOS = 0.0;

  // Soft limits (in degrees)
  public static final double HOOD_MIN_ANGLE = 33.4;
  public static final double HOOD_MAX_ANGLE = 50;

  // Tolerance for reaching target angle (in degrees)
  public static final Rotation2d HOOD_ANGLE_TOLERANCE = Rotation2d.fromDegrees(2.0);
  public static final Rotation2d HOOD_IZONE = Rotation2d.fromDegrees(10.0);

  // Absolute encoder offset (in degrees)
  // This is the angle reading when the hood is at 0 degrees
  public static final Rotation2d ENCODER_OFFSET = Rotation2d.fromRotations(0.0);

  public static final NavigableMap<Double, Double> DISTANCE_TO_ANGLE_MAP = new TreeMap<>();

  static {
    DISTANCE_TO_ANGLE_MAP.put(Units.inchesToMeters(53.8100505), HOOD_MIN_ANGLE);
    DISTANCE_TO_ANGLE_MAP.put(Units.inchesToMeters(62.0600505), HOOD_MIN_ANGLE);
    DISTANCE_TO_ANGLE_MAP.put(Units.inchesToMeters(83.3100505), HOOD_MIN_ANGLE);
    DISTANCE_TO_ANGLE_MAP.put(Units.inchesToMeters(109.8100505), HOOD_MIN_ANGLE);
    DISTANCE_TO_ANGLE_MAP.put(Units.inchesToMeters(114.8100505), HOOD_MIN_ANGLE);
  }
}
