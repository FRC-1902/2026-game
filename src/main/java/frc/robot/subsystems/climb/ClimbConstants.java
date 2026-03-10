package frc.robot.subsystems.climb;

public class ClimbConstants {
  public static final int LIMIT_SWITCH_PORT = 0; // TODO: Update port binding

  // PID
  public static final double CLIMB_KP = 0.0; // TODO: Tune PID
  public static final double CLIMB_KI = 0.0;
  public static final double CLIMB_KD = 0.0;

  public static final int CLIMB_MOTOR_ID = 6; // TODO: Update motor CAN ID

  public static final double CLIMB_IZONE = 0.0; // TODO: TUNE TUNE TUNE
  public static final double CLIMB_TOLERANCE = 0.0; // TODO: TUNE!!!!!

  // Feedforward constants
  public static final double CLIMB_HOLDING_KCOS = 0.0; // TODO: Tune for holding elevator (DOWN/UP)
  public static final double CLIMB_RELEASING_KCOS = 0.0; // TODO: Tune for releasing (heavier FF)

  // Minimum allowed elevator height (unit: rotations)
  public static final double CLIMB_MIN_HEIGHT = 0.0; // TODO: Set to encoder value at bottom
  public static final double CLIMB_MAX_HEIGHT = 1.0; // TODO: Set to encoder value at top

  // Climb speed (unit: percent output [-1.0, 1.0])
  public static final double CLIMB_CLIMBING_SPEED = 0.8; // TODO: Tune for safe climbing speed
}
