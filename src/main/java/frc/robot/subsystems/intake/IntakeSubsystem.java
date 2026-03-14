package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class IntakeSubsystem extends SubsystemBase {

  private final SparkMax rollerMotor;
  private final SparkMax pivotMotor;
  private final SparkAbsoluteEncoder pivotEncoder;

  private Rotation2d targetAngle = new Rotation2d();

  PIDController pid =
      new PIDController(
          IntakeConstants.INTAKE_KP, IntakeConstants.INTAKE_KI, IntakeConstants.INTAKE_KD);

  public IntakeSubsystem() {

    pid.enableContinuousInput(IntakeConstants.DISABLED_INTAKE_ANGLE, IntakeConstants.ENABLED_INTAKE_ANGLE);
    pid.setIZone(IntakeConstants.PID_IZONE);
    pid.setTolerance(IntakeConstants.PID_TOLERANCE);

    pivotMotor = new SparkMax(IntakeConstants.PIVOTMOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    rollerMotor = new SparkMax(IntakeConstants.ROLLERMOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    pivotEncoder = pivotMotor.getAbsoluteEncoder();
    // Set up motors, encoder & PID

    SparkMaxConfig pivotConfig = new SparkMaxConfig();
    pivotConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
    pivotConfig.smartCurrentLimit(IntakeConstants.PIVOTMOTOR_CURRENTLIMIT);
    pivotConfig.voltageCompensation(IntakeConstants.PIVOTMOTOR_VOLTAGECOMPENSATION);
    pivotMotor.configure(
        pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig rollerConfig = new SparkMaxConfig();
    rollerConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
    rollerConfig.smartCurrentLimit(IntakeConstants.ROLLERMOTOR_CURRENTLIMIT);
    rollerConfig.voltageCompensation(IntakeConstants.ROLLERMOTOR_VOLTAGECOMPENSATION);
    rollerMotor.configure(
        rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // Setup PID feedback loop & configs
  }

  private double calculateGravityFeedforward(Rotation2d angle) {
    return IntakeConstants.INTAKE_KG * Math.cos(angle.getRadians());
  }

public Rotation2d getAngle() {
  double rawRotations = pivotEncoder.getPosition();
  double offsetRotations = IntakeConstants.ENCODER_OFFSET.getRotations();
  double mechanismRotations = (rawRotations - offsetRotations) * IntakeConstants.ENCODER_TO_INTAKE_RATIO;
  return Rotation2d.fromRotations(mechanismRotations);
}

  public void setAngle(Rotation2d angle) {
    targetAngle = angle;
  }

  public void setIntakeState(boolean state) {
    Rotation2d angle = Rotation2d.fromDegrees(IntakeConstants.ENABLED_INTAKE_ANGLE);
    if (!state) {
      angle = Rotation2d.fromDegrees(IntakeConstants.DISABLED_INTAKE_ANGLE);
    }
    // Set the target angle to be the angle of an enabled intake
    if (targetAngle != angle) {
      setAngle(angle);
    }
    // Set the angle of the pivotMotor to that angle
  }

  public boolean getIntakeState() {
    if (getAngle().getDegrees() == IntakeConstants.DISABLED_INTAKE_ANGLE) {
      return false;
    } else {
      return true;
    }
  }

  public void startRollers() {
    // rollerMotor.set(IntakeConstants.ROLLERMOTOR_SPEED);
    // TODO: re-enable motors
    rollerMotor.set(1);
  }

  public void stopRollers() {
    rollerMotor.set(0);
  }

  public boolean atSetpoint() {
    return pid.atSetpoint();
  }

  @Override
  public void periodic() {
    double measurement = getAngle().getDegrees();
    double setpoint = targetAngle.getDegrees();
    double pidoutput = pid.calculate(measurement, setpoint);
    double ff = calculateGravityFeedforward(getAngle());
    double output = pidoutput + ff;
    output = Math.max(-1.0, Math.min(1.0, output));
    // pivotMotor.set(output);
    pivotMotor.set(output);
    // TODO: re-enable motors
    // Calculates PID and sets pivotMotor to it

    SmartDashboard.putNumber("Intake/Current Pivot Angle", getAngle().getDegrees());
    SmartDashboard.putNumber("Intake/Target Pivot Angle", targetAngle.getDegrees());
    SmartDashboard.putNumber("Output", output);
    // Add all values to network table
  }

  public Command EnableIntakeCommand() {
    return this.runOnce(() -> setIntakeState(true))
        .alongWith(new WaitUntilCommand(this::atSetpoint));
  }

  public Command disableIntakeCommand() {
    return this.runOnce(() -> setIntakeState(false))
        .alongWith(new WaitUntilCommand(this::atSetpoint));
  }

  public Command StartRollersCommand() {
    return this.startEnd(() -> startRollers(), () -> stopRollers());
  }
}
