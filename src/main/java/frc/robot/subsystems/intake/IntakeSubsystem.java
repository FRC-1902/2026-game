package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private final SparkMax rollerMotor;
  private final SparkMax pivotMotor;
  private final SparkAbsoluteEncoder pivotEncoder;

  private Rotation2d targetAngle = new Rotation2d();

  PIDController pid =
      new PIDController(
          IntakeConstants.INTAKE_KP, IntakeConstants.INTAKE_KI, IntakeConstants.INTAKE_KD);

  public IntakeSubsystem() {

    pid.setIZone(IntakeConstants.PID_IZONE);
    pid.setTolerance(IntakeConstants.PID_TOLERANCE);

    pivotMotor = new SparkMax(IntakeConstants.PIVOTMOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    rollerMotor = new SparkMax(IntakeConstants.ROLLERMOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    pivotEncoder = pivotMotor.getAbsoluteEncoder();
    // Set up motors, encoder & PID

    SparkMaxConfig config = new SparkMaxConfig();
    pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rollerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // Setup PID feedback loop & configs
  }

  private double calculateGravityFeedforward(Rotation2d angle) {
    return IntakeConstants.INTAKE_KG * Math.cos(angle.getRadians());
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(
        pivotEncoder.getPosition() * IntakeConstants.ENCODER_TO_INTAKE_RATIO
            - IntakeConstants.ENCODER_OFFSET);
    // Get the angle of the encoder
  }

  public void setAngle(Rotation2d angle) {
    targetAngle = angle;
    double doubleAngle = angle.getDegrees();
    double ffVolts = calculateGravityFeedforward(angle);
    pivotMotor
        .getClosedLoopController()
        .setSetpoint(
            doubleAngle / IntakeConstants.MOTOR_TO_INTAKE_RATIO,
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            ffVolts);
  }

  public void setIntakeState(boolean state) {
    Rotation2d angle = Rotation2d.fromDegrees(IntakeConstants.ENABLED_INTAKE_ANGLE);
    if (!state) {
      angle = Rotation2d.fromDegrees(IntakeConstants.DISABLED_INTAKE_ANGLE);
    }
    // Set the target angle to be the angle of an enabled intake
    setAngle(angle);
    // Set the angle of the pivotMotor to that angle
  }

  public void startRollers() {
    rollerMotor.set(IntakeConstants.ROLLERMOTOR_SPEED);
  }

  public void stopRollers() {
    rollerMotor.stopMotor();
  }

  public void periodic() {

    double measurement = getAngle().getDegrees();
    double setpoint = targetAngle.getDegrees();
    double pidoutput = pid.calculate(measurement, setpoint);
    double ff = calculateGravityFeedforward(getAngle());
    double output = pidoutput + ff;
    output = Math.max(-1.0, Math.min(1.0, output));
    pivotMotor.set(output);
    // Calculates PID and sets pivotMotor to it

    SmartDashboard.putNumber("Intake/Current Pivot Angle", getAngle().getDegrees());
    SmartDashboard.putNumber("Intake/Target Pivot Angle", targetAngle.getDegrees());
    // Add all values to network table
  }
}
