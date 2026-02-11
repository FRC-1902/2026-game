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
  private boolean intakeEnabled = false;

  PIDController pid =
      new PIDController(
          IntakeConstants.INTAKE_KP, IntakeConstants.INTAKE_KI, IntakeConstants.INTAKE_KD);

  public IntakeSubsystem() {

    pivotMotor = new SparkMax(IntakeConstants.PIVOTMOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    rollerMotor = new SparkMax(IntakeConstants.ROLLERMOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    pivotEncoder = pivotMotor.getAbsoluteEncoder();

    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop.pid(
        IntakeConstants.INTAKE_KP, IntakeConstants.INTAKE_KI, IntakeConstants.INTAKE_KD);
    config.closedLoop.iZone(0);
    config.closedLoop.outputRange(-1.0, 1.0);
    pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // Setup PID feedback loop and configs
  }

  private double calculateGravityFeedforward(Rotation2d angle) {
    return IntakeConstants.INTAKE_KG * Math.cos(angle.getRadians());
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromRadians(
        pivotEncoder.getPosition() * IntakeConstants.MOTOR_TO_INTAKE_RATIO);
    // Get the angle of the encoder in degrees.
  }

  public void setAngle(Rotation2d angle) {
    double doubleAngle = angle.getDegrees();
    double ffVolts = calculateGravityFeedforward(angle);
    pivotMotor
        .getClosedLoopController()
        .setSetpoint(
            doubleAngle * IntakeConstants.MOTOR_TO_INTAKE_RATIO,
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            ffVolts);
  }

  public void intakeDown() {
    targetAngle = Rotation2d.fromDegrees(IntakeConstants.ENABLED_INTAKE_ANGLE);
    // Set the target angle to be the angle of an enabled intake
    setAngle(targetAngle);
    // Set the angle of the pivotMotor to that angle
    intakeEnabled = true;
  }

  public void startRollers() {
    rollerMotor.set(1.0);
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
    // Calculate PID and set pivotMotor to it

    SmartDashboard.putNumber("Intake/Current Pivot Angle", getAngle().getDegrees());
    SmartDashboard.putNumber("Intake/Target Pivot Angle", targetAngle.getDegrees());
    SmartDashboard.putBoolean("Intake/Intake Enabled", intakeEnabled);
    // Add all values to network table
  }
}
