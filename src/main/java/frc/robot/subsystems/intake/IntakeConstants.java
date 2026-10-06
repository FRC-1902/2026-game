package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;

public class IntakeConstants {

  public static final Rotation2d DISABLED_INTAKE_ANGLE = Rotation2d.fromDegrees(172.5);
  public static final Rotation2d ENABLED_INTAKE_ANGLE = Rotation2d.fromDegrees(82);
  public static final Rotation2d INDEXING_OSCILLATION_UP_ANGLE = Rotation2d.fromDegrees(120);

  // CAN IDs

  public static final int ROLLERMOTOR_ID = 1;
  public static final int PIVOTMOTOR_ID = 4;

  // Encoder

  public static final double ENCODER_TO_INTAKE_RATIO = 0.5; // 1:2
  public static final double ENCODER_OFFSET = 0.5;

  // PID

  public static final double INTAKE_KP = 0.008;
  public static final double INTAKE_KI = 0.0;
  public static final double INTAKE_KD = 0.0;

  public static final Rotation2d PID_TOLERANCE = Rotation2d.fromDegrees(5.0);
  public static final Rotation2d PID_IZONE = Rotation2d.fromDegrees(10.0);

  // FF

  public static final double INTAKE_KG = 0.04;

  // Motors

  public static final double ROLLERMOTOR_SPEED = 0.75;
  public static final double INDEXING_OSCILLATION_RISE_TIME_SECONDS = 3.0;
  public static final double INDEXING_OSCILLATION_DROP_TIME_SECONDS = 3.0;

  public static final int ROLLERMOTOR_CURRENTLIMIT = 40;
  public static final int PIVOTMOTOR_CURRENTLIMIT = 40;

  public static final double ROLLERMOTOR_VOLTAGECOMPENSATION = 12.0;
  public static final double PIVOTMOTOR_VOLTAGECOMPENSATION = 12.0;
}
