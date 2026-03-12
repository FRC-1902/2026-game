package frc.robot.subsystems.intake;

public class IntakeConstants {

  public static final double DISABLED_INTAKE_ANGLE =
      0.0; // TODO: get proper angle for intake whilst up
  public static final double ENABLED_INTAKE_ANGLE =
      0.0; // TODO: get proper angle for intake whilst down

  // IDs/Ports

  public static final int INTAKE_ENCODER_PORT = 1; // TODO: set correct port

  public static final int ROLLERMOTOR_ID = 1; // TODO: set correct port
  public static final int PIVOTMOTOR_ID = 4; // TODO: set correct port

  // Gear ratios

  public static final double MOTOR_TO_INTAKE_RATIO = 18.0; // 18:1
  public static final double INTAKE_TO_ENCODER_RATIO = 2.0; // 2:1
  public static final double ENCODER_TO_INTAKE_RATIO = 0.5; // 1:2

  public static final double INTAKE_KG = 1.0; // TODO: get real value

  // PID

  public static final double PID_IZONE = 0.0; // TODO: get real values
  public static final double PID_TOLERANCE = 0.0; // TODO: get real tolerance

  public static final double INTAKE_KP = 2.0; // TODO: tune PID values
  public static final double INTAKE_KI = 2.0;
  public static final double INTAKE_KD = 2.0;

  public static final double ENCODER_OFFSET =
      0.0; // TODO: set based on calibration (must be in rotations)

  // Config

  public static final double IZONE = 0.0; // TODO: tune

  // Motors

  public static final double ROLLERMOTOR_SPEED = 1.0; // TODO: tune

  public static final int ROLLERMOTOR_CURRENTLIMIT = 40; // TODO: ~Temporary value
  public static final int PIVOTMOTOR_CURRENTLIMIT = 30; // TODO: ~Temporary value

  public static final double ROLLERMOTOR_VOLTAGECOMPENSATION = 12.0;
  public static final double PIVOTMOTOR_VOLTAGECOMPENSATION = 12.0;
}
