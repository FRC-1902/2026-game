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
  private final PIDController pid;

  private Rotation2d targetAngle = IntakeConstants.DISABLED_INTAKE_ANGLE;

  public IntakeSubsystem() {
    // setup pid
    pid =
        new PIDController(
            IntakeConstants.INTAKE_KP, IntakeConstants.INTAKE_KI, IntakeConstants.INTAKE_KD);
    pid.enableContinuousInput(0, 360);
    pid.setIZone(IntakeConstants.PID_IZONE.getDegrees());
    pid.setTolerance(IntakeConstants.PID_TOLERANCE.getDegrees());

    // construct motors + encoders
    pivotMotor = new SparkMax(IntakeConstants.PIVOTMOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    rollerMotor = new SparkMax(IntakeConstants.ROLLERMOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    pivotEncoder = pivotMotor.getAbsoluteEncoder();

    // configure motors
    SparkMaxConfig pivotConfig = new SparkMaxConfig();
    pivotConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
    pivotConfig.smartCurrentLimit(IntakeConstants.PIVOTMOTOR_CURRENTLIMIT);
    pivotConfig.voltageCompensation(IntakeConstants.PIVOTMOTOR_VOLTAGECOMPENSATION);

    // TODO: clean this up after orlando
    pivotConfig.apply(pivotConfig.absoluteEncoder.zeroOffset(0.6));

    pivotMotor.configure(
        pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig rollerConfig = new SparkMaxConfig();
    rollerConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
    rollerConfig.smartCurrentLimit(IntakeConstants.ROLLERMOTOR_CURRENTLIMIT);
    rollerConfig.voltageCompensation(IntakeConstants.ROLLERMOTOR_VOLTAGECOMPENSATION);
    rollerMotor.configure(
        rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private double calculateGravityFeedforward(Rotation2d angle) {
    return IntakeConstants.INTAKE_KG * Math.cos(angle.getRadians());
  }

  public Rotation2d getAngle() {
    double rotations = pivotEncoder.getPosition();
    rotations *= IntakeConstants.ENCODER_TO_INTAKE_RATIO;

    // TODO: clean this up after orlando
    rotations -= (0.01 / 360.0);

    return Rotation2d.fromRotations(rotations);
  }

  private void setAngle(Rotation2d angle) {
    targetAngle = angle;
  }

  public void setIntakeState(boolean state) {
    if (state) {
      setAngle(IntakeConstants.ENABLED_INTAKE_ANGLE);
    } else {
      setAngle(IntakeConstants.DISABLED_INTAKE_ANGLE);
    }
  }

  public void startRollers() {
    rollerMotor.set(IntakeConstants.ROLLERMOTOR_SPEED);
  }

  public void stopRollers() {
    rollerMotor.set(0);
  }

  public boolean atSetpoint() {
    return pid.atSetpoint();
  }

  public Command enableIntakeCommand() {
    return this.runOnce(() -> setIntakeState(true)).andThen(new WaitUntilCommand(this::atSetpoint));
  }

  public Command disableIntakeCommand() {
    return this.runOnce(() -> setIntakeState(false))
        .andThen(new WaitUntilCommand(this::atSetpoint));
  }

  public Command startRollersCommand() {
    return this.startEnd(() -> startRollers(), () -> stopRollers());
  }

  public Command toggleRollersOnCommand() {
    return this.run(() -> startRollers());
  }

  public Command toggleRollersOffCommand() {
    return this.run(() -> stopRollers());
  }

  @Override
  public void periodic() {
    // calculate pid + ff for pivot motor
    Rotation2d measurement = getAngle();
    double setpoint = targetAngle.getDegrees();
    double pidoutput = pid.calculate(measurement.getDegrees(), setpoint);
    double ff = calculateGravityFeedforward(measurement);
    double output = pidoutput + ff;
    output = Math.max(-0.2, Math.min(0.4, output));
    pivotMotor.set(output);

    // add all values to network table
    SmartDashboard.putNumber("Intake/Current Pivot Angle", measurement.getDegrees());
    SmartDashboard.putNumber("Intake/Target Pivot Angle", setpoint);
    SmartDashboard.putNumber("Intake/Output Percent", output);
  }
}
