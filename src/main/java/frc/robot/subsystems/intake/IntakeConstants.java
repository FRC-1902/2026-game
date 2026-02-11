package frc.robot.subsystems.intake;

public class IntakeConstants {

  public static final double DISABLED_INTAKE_ANGLE =
      0.0; // TODO: get proper angle for intake whilst up
  public static final double ENABLED_INTAKE_ANGLE =
      0.0; // TODO: get proper angle for intake whilst down

  // IDs/Ports

  public static final int INTAKE_ENCODER_PORT = 0; // TODO: set correct port

  public static final int ROLLERMOTOR_ID = 0; // TODO: set correct port
  public static final int PIVOTMOTOR_ID = 0; // TODO: set correct port

  // Gear ratios

  public static final double MOTOR_TO_CENTER_AXLE_RATIO = 9.0; // 9:1
  public static final double CENTER_AXLE_TO_INTAKE_RATIO = 2.0; // 2:1
  public static final double MOTOR_TO_INTAKE_RATIO =
      MOTOR_TO_CENTER_AXLE_RATIO * CENTER_AXLE_TO_INTAKE_RATIO; // 18:1

  public static final double INTAKE_TO_ENCODER_RATIO = 2.0; // 2:1

  public static final double INTAKE_KG = 3.0; // TODO: get real weight

  // PID

  public static final double INTAKE_KP = 2.0; // TODO: tune PID values
  public static final double INTAKE_KI = 2.0;
  public static final double INTAKE_KD = 2.0;

  public static final double ENCODER_OFFSET = 0.0; // TODO: set based on calibration

  // Config

  public static final double IZONE = 0.0; // TODO: tune

  // Motors

  public static final double ROLLERMOTOR_SPEED = 1.0; // TODO: tune
}
