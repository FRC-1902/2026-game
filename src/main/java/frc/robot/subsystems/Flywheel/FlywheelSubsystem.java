package frc.robot.subsystems.Flywheel;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlywheelSubsystem extends SubsystemBase {
  double targetRpm;

  private final SparkMax leftFlywheelMotor;
  private final SparkMax rightFlywheelMotor;
  private boolean toggleState;
  private final PIDController pidController;

  public FlywheelSubsystem() {
    // Initialize motor
    rightFlywheelMotor =
        new SparkMax(FlywheelConstants.RIGHT_FLYWHEEL_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    leftFlywheelMotor =
        new SparkMax(FlywheelConstants.LEFT_FLYWHEEL_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

    toggleState = false;

    // Configure motor
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(SparkBaseConfig.IdleMode.kCoast);
    config.inverted(true);
    config.smartCurrentLimit(50);

    rightFlywheelMotor.configure(
        config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    config.inverted(false);
    leftFlywheelMotor.configure(
        config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pidController =
        new PIDController(
            FlywheelConstants.FLYWHEEL_KP,
            FlywheelConstants.FLYWHEEL_KI,
            FlywheelConstants.FLYWHEEL_KD);

    double tol = FlywheelConstants.RPM_TOLERANCE;
    pidController.setTolerance(tol);

    SmartDashboard.putNumber("Flywheel/Setpoint RPM", 0);
  }

  public void setSpeed(double s) {
    leftFlywheelMotor.set(s);
    rightFlywheelMotor.set(s);
  }

  public void setFlywheelSpeed(double rpm) {
    targetRpm = rpm;
  }

  public double getFlywheelSpeed() {
    // Uses the right side as a reference;
    return (rightFlywheelMotor.getEncoder().getVelocity())
        * FlywheelConstants.MOTOR_TO_FLYWHEEL_RATIO;
  }

  public void spinToHighSpeed() {
    setFlywheelSpeed(FlywheelConstants.DESIRED_FLYWHEEL_RPM);
  }

  public void spinDownToZero() {
    setFlywheelSpeed(0);
  }

  public boolean isAtTargetSpeed() {
    return Math.abs(getFlywheelSpeed() - targetRpm) <= FlywheelConstants.RPM_TOLERANCE;
  }

  public void toggle() {
    toggleState = !toggleState;

    if (toggleState) {
      setSpeed(1);
    } else {
      setSpeed(0);
    }
  }

  public void spinFlywheelBackwards() {
    setFlywheelSpeed(-FlywheelConstants.DESIRED_LOW_FLYWHEEL_RPM);
  }

  public Command testCommand() {
    return startEnd(() -> setSpeed(1), () -> setSpeed(0));
  }

  @Override
  public void periodic() {
    // double currentRpm = getFlywheelSpeed();
    // pidController.setP(FlywheelConstants.FLYWHEEL_KP);
    // pidController.setI(FlywheelConstants.FLYWHEEL_KI);
    // pidController.setD(FlywheelConstants.FLYWHEEL_KD);

    // double pidOutput = pidController.calculate(currentRpm, targetRpm);

    // pidOutput = Math.max(-1.0, Math.min(1.0, pidOutput));
    // if (Math.abs(targetRpm) < 1e-6) {
    //   setSpeed(0);
    // } else {
    //   setSpeed(pidOutput);
    // }

    // Update dashboard
    // SmartDashboard.putNumber("Flywheel/ Speed", currentRpm);
    SmartDashboard.putNumber("Flywheel/Target RPM", targetRpm);
    // SmartDashboard.putNumber("Flywheel/PID Output", pidOutput);
    SmartDashboard.putNumber("Flywheel/ right current draw", rightFlywheelMotor.getOutputCurrent());
    SmartDashboard.putNumber("Flywheel/ left current draw", leftFlywheelMotor.getOutputCurrent());
    SmartDashboard.putBoolean("Flywheel/At Target Speed", isAtTargetSpeed());
  }
}
