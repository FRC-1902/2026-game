package frc.robot.subsystems.Flywheel;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Note: We intentionally avoid a hard dependency on SysIdRoutine's constructor here because
// different WPILib versions expose different constructors/factories. The subsystem provides
// `setVoltage(double)` and `getMeasurement()` which are the required callbacks SysId needs.

public class FlywheelSubsystem extends SubsystemBase {

  double currentSpeed;
  private final SparkMax leftFlywheelMotor;
  private final SparkMax rightFlywheelMotor;
  private final PIDController pid =
      new PIDController(
          FlywheelConstants.FLYWHEEL_KP,
          FlywheelConstants.FLYWHEEL_KI,
          FlywheelConstants.FLYWHEEL_KD);

  public FlywheelSubsystem() {
    // Initialize motor
    rightFlywheelMotor =
        new SparkMax(FlywheelConstants.RIGHT_FLYWHEEL_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    leftFlywheelMotor =
        new SparkMax(FlywheelConstants.LEFT_FLYWHEEL_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

    // Configure motor
    SparkMaxConfig config = new SparkMaxConfig();

    rightFlywheelMotor.configure(
        config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftFlywheelMotor.configure(
        config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setFlywheelSpeed(double speed) {
    leftFlywheelMotor.set(-speed);
    rightFlywheelMotor.set(speed);
  }

  public double getFlywheelSpeed() {
    // Uses the right side as a reference;
    return (rightFlywheelMotor.getEncoder().getVelocity()) * 2;
  }

  public void spinUpToSpeed() {
    double output =
        pid.calculate(
            currentSpeed,
            FlywheelConstants.DESIRED_FLYWHEEL_RPM); // TODO: Add the SysId into the calculation
    setFlywheelSpeed(output);
  }

  public void spinDownToLowSpeed() {
    double output =
        pid.calculate(
            currentSpeed,
            FlywheelConstants.DESIRED_LOW_FLYWHEEL_RPM); // TODO: Add the SysId into the calculation
    setFlywheelSpeed(output);
  }

  public void spinDownToZero() {
    double output = pid.calculate(currentSpeed, 0); // TODO: Add the SysId into the calculation
    setFlywheelSpeed(output);
  }

  public boolean isAtTargetSpeed() {
    return currentSpeed
        >= FlywheelConstants.DESIRED_FLYWHEEL_RPM
            * 0.95; // Consider it at target if it's within 5% of the desired RPM
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    currentSpeed = getFlywheelSpeed();
    SmartDashboard.putNumber("Flywheel/ Speed", getFlywheelSpeed());
    SmartDashboard.putNumber("Flywheel/Target RPM", FlywheelConstants.DESIRED_FLYWHEEL_RPM);
    SmartDashboard.putBoolean("Flywheel/At Target Speed", isAtTargetSpeed());
  }
}
