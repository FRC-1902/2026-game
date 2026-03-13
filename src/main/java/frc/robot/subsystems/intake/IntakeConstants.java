package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;

public class IntakeConstants {

  // Angles are expressed in rotations (turns). Use Rotation2d.fromRotations(value).
  public static final Rotation2d DISABLED_INTAKE_ANGLE = Rotation2d.fromRotations(0.0);
  public static final Rotation2d ENABLED_INTAKE_ANGLE = Rotation2d.fromRotations(0.2337);

  // IDs/Ports

  public static final int ROLLERMOTOR_ID = 1;
  public static final int PIVOTMOTOR_ID = 4;

  // Gear ratios

  public static final double MOTOR_TO_INTAKE_RATIO = 18.0; // 18:1
  public static final double INTAKE_TO_ENCODER_RATIO = 2.0; // 2:1
  public static final double ENCODER_TO_INTAKE_RATIO = 0.5; // 1:2

  public static final double INTAKE_KG = 0.0; // TODO: get real value

  // PID

  public static final double PID_IZONE = 0.0; // TODO: get real values
  public static final double PID_TOLERANCE = 0.0; // TODO: get real tolerance

  public static final double INTAKE_KP = 0.001; // TODO: tune PID values
  public static final double INTAKE_KI = 0.0;
  public static final double INTAKE_KD = 0.0;

  // Encoder offset must be provided in rotations (turns).
  public static final Rotation2d ENCODER_OFFSET =
      Rotation2d.fromRotations(0.3237); // TODO: set based on calibration (rotations)

  // Config

  public static final double IZONE = 0.0; // TODO: tune

  // Motors

  public static final double ROLLERMOTOR_SPEED = 1.0; // TODO: tune

  public static final int ROLLERMOTOR_CURRENTLIMIT = 40; // TODO: ~Temporary value
  public static final int PIVOTMOTOR_CURRENTLIMIT = 30; // TODO: ~Temporary value

  public static final double ROLLERMOTOR_VOLTAGECOMPENSATION = 12.0;
  public static final double PIVOTMOTOR_VOLTAGECOMPENSATION = 12.0;
}
