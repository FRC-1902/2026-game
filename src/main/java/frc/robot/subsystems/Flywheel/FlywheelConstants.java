package frc.robot.subsystems.Flywheel;

/** Constants for the Hood subsystem. */
public class FlywheelConstants {
  // Motor CAN ID
  public static final int RIGHT_FLYWHEEL_MOTOR_ID = 34;
  public static final int LEFT_FLYWHEEL_MOTOR_ID = 39;

  // PID
  public static final double FLYWHEEL_KP = 0.0; // TODO: after bayou, add in a pid to make spin up faster
  public static final double FLYWHEEL_KI = 0.0;
  public static final double FLYWHEEL_KD = 0.0;

  // Gear ratio for the motor to the flywheel. This is used to convert between motor RPM and
  // flywheel RPM.
  public static final double MOTOR_TO_FLYWHEEL_RATIO = 2.0; // 2:1

  // Desired flywheel RPM for shooting. This is the target speed we want the flywheel to reach when
  // shooting.
  public static final double DESIRED_FLYWHEEL_RPM = 5000.0;
  public static final double DESIRED_LOW_FLYWHEEL_RPM = 0.0; // TODO: Set the correct low RPM for the flywheel when not shooting.
  public static final double RPM_TOLERANCE = 50.0;

  public static final double FLYWHEEL_KS = 0.284;
  public static final double FLYWHEEL_KV = 0.12516;
  public static final double FLYWHEEL_KA = 0.01736;
}
