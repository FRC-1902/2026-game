package frc.robot.subsystems.Flywheel;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Note: We intentionally avoid a hard dependency on SysIdRoutine's constructor here because
// different WPILib versions expose different constructors/factories. The subsystem provides
// `setVoltage(double)` and `getMeasurement()` which are the required callbacks SysId needs.

public class FlywheelSubsystem extends SubsystemBase {
  double targetRpm;

  private final SparkMax leftFlywheelMotor;
  private final SparkMax rightFlywheelMotor;
  private boolean toggleState;

  private final PIDController pid =
      new PIDController(
          FlywheelConstants.FLYWHEEL_KP,
          FlywheelConstants.FLYWHEEL_KI,
          FlywheelConstants.FLYWHEEL_KD);

  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          FlywheelConstants.FLYWHEEL_KS,
          FlywheelConstants.FLYWHEEL_KV,
          FlywheelConstants.FLYWHEEL_KA);

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
  // methods for spinning the flywheel up and down
  public void spinToLowSpeed() {
    setFlywheelSpeed(FlywheelConstants.DESIRED_LOW_FLYWHEEL_RPM);
  }

  public void spinDownToZero() {
    setFlywheelSpeed(0);
  }

  public void setFlywheelVoltage(double volatage) {
    rightFlywheelMotor.setVoltage(volatage);
  }

  public boolean isAtTargetSpeed() {
    return Math.abs(getFlywheelSpeed() - targetRpm) <= FlywheelConstants.RPM_TOLERANCE;
  }

  public void toggle() {
    toggleState = !toggleState;

    if (toggleState) {
      spinToHighSpeed();
    } else if (!toggleState) {
      spinToLowSpeed();
    } else {
      spinDownToZero();
    }
  }

  public void spinFlywheelBackwards() {
    setFlywheelSpeed(-FlywheelConstants.DESIRED_LOW_FLYWHEEL_RPM);
  }

  @Override
  public void periodic() {
    double output = 0;
    if (targetRpm > 0.001 || targetRpm < -0.001) {
      output += feedforward.calculate(targetRpm);
      output += pid.calculate(getFlywheelSpeed(), targetRpm);
    } else {
      output = 0;
    }

    leftFlywheelMotor.setVoltage(output);
    rightFlywheelMotor.setVoltage(-output);

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Flywheel/ Speed", getFlywheelSpeed());
    SmartDashboard.putNumber("Flywheel/Target RPM", targetRpm);
    SmartDashboard.putNumber("Flywheel/ right current draw", rightFlywheelMotor.getOutputCurrent());
    SmartDashboard.putNumber("Flywheel/ left current draw", leftFlywheelMotor.getOutputCurrent());
    SmartDashboard.putBoolean("Flywheel/At Target Speed", isAtTargetSpeed());
  }
}
