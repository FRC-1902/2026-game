package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;

public class IntakeConstants {

  // TODO: get proper angles for intake
  public static final Rotation2d DISABLED_INTAKE_ANGLE = Rotation2d.fromDegrees(0);
  public static final Rotation2d ENABLED_INTAKE_ANGLE = Rotation2d.fromDegrees(0);

  // CAN IDs

  public static final int ROLLERMOTOR_ID = 1; // TODO: set correct port
  public static final int PIVOTMOTOR_ID = 4; // TODO: set correct port

  // Encoder

  public static final double ENCODER_TO_INTAKE_RATIO = 0.5; // 1:2
  public static final Rotation2d ENCODER_OFFSET = Rotation2d.fromDegrees(0.0);

  // PID

  // TODO: tune PID values
  public static final double INTAKE_KP = 0.001;
  public static final double INTAKE_KI = 0.0;
  public static final double INTAKE_KD = 0.0;

  public static final Rotation2d PID_TOLERANCE = Rotation2d.fromDegrees(2.0);
  public static final Rotation2d PID_IZONE = Rotation2d.fromDegrees(10.0);

  // FF

  public static final double INTAKE_KG = 0.0; // TODO: get real value

  // Motors

  public static final double ROLLERMOTOR_SPEED = 1.0; // TODO: tune

  public static final int ROLLERMOTOR_CURRENTLIMIT = 40;
  public static final int PIVOTMOTOR_CURRENTLIMIT = 40;

  public static final double ROLLERMOTOR_VOLTAGECOMPENSATION = 12.0;
  public static final double PIVOTMOTOR_VOLTAGECOMPENSATION = 12.0;
}
