package frc.robot.subsystems.Flywheel;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlywheelSubsystem extends SubsystemBase {
  double targetRpm;

  private final BangBangController bangBangController;
  private double toleranceRpm = 10.0;

  private final SparkMax leftFlywheelMotor;
  private final SparkMax rightFlywheelMotor;
  private boolean toggleState;

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

  bangBangController = new BangBangController();
  }

  public void setSpeed(double s) {
    leftFlywheelMotor.set(s);
    rightFlywheelMotor.set(s);
  }

  public double getFlywheelSpeed() {
    // Uses the right side as a reference;
    return (rightFlywheelMotor.getEncoder().getVelocity())
        * FlywheelConstants.MOTOR_TO_FLYWHEEL_RATIO;
  }

  public void setTargetRpm(double rpm) {
    targetRpm = rpm;
  }

  public void stop() {
    targetRpm = 0;
    setSpeed(0);
  }

  public boolean isAtTarget() {
    if (targetRpm <= 0) {
      return false;
    }
    return Math.abs(getFlywheelSpeed() - targetRpm) <= toleranceRpm;
  }

  public boolean isEnabled() {
    return toggleState;
  }

  public boolean isStopped() {
    return Math.abs(getFlywheelSpeed()) <= toleranceRpm;
  }

  public void toggle() {
    toggleState = !toggleState;

    if (toggleState) {
      setTargetRpm(FlywheelConstants.DESIRED_FLYWHEEL_RPM);
    } else {
      stop();
    }
  }

  public Command spinFlywheel() {
    return Commands.startEnd(() -> setSpeed(1), () -> setSpeed(0), this);
  }

  public Command spinFlywheelToRpm(double rpm) {
    return Commands.startEnd(() -> setTargetRpm(rpm), this::stop, this);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Flywheel/Target RPM", targetRpm);
    SmartDashboard.putNumber("Flywheel/ right current draw", rightFlywheelMotor.getOutputCurrent());
    SmartDashboard.putNumber("Flywheel/ left current draw", leftFlywheelMotor.getOutputCurrent());
    SmartDashboard.putNumber("Flywheel/ RPM", getFlywheelSpeed());

    if (targetRpm > 0) {
      double output = bangBangController.calculate(getFlywheelSpeed(), targetRpm);
      setSpeed(output);
      SmartDashboard.putNumber("Flywheel/Output", output);
      SmartDashboard.putBoolean("Flywheel/AtTarget", isAtTarget());
    }
  }
}
