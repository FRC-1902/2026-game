package frc.robot.subsystems.Flywheel;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

// Note: We intentionally avoid a hard dependency on SysIdRoutine's constructor here because
// different WPILib versions expose different constructors/factories. The subsystem provides
// `setVoltage(double)` and `getMeasurement()` which are the required callbacks SysId needs.

public class FlywheelSubsystem extends SubsystemBase {
  boolean on;

  private final SparkMax leftFlywheelMotor;
  private final SparkMax rightFlywheelMotor;

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

  private DoubleSupplier distanceToHub;

  public FlywheelSubsystem(DoubleSupplier distanceToHub) {
    this.distanceToHub = distanceToHub;

    // Initialize motor
    rightFlywheelMotor =
        new SparkMax(FlywheelConstants.RIGHT_FLYWHEEL_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    leftFlywheelMotor =
        new SparkMax(FlywheelConstants.LEFT_FLYWHEEL_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

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

  private void setVoltage(Voltage v) {
    leftFlywheelMotor.setVoltage(v);
    rightFlywheelMotor.setVoltage(v);
  }

  public void setState(boolean on) {
    this.on = on;
  }

  public double getMotorSpeed() {
    // Uses the right side as a reference;
    return rightFlywheelMotor.getEncoder().getVelocity();
  }

  public boolean isAtTargetSpeed() {
    double targetRpm;
    if (this.on) {
      targetRpm = getRpmForDistance(distanceToHub.getAsDouble());
    } else {
      targetRpm = 0;
    }
    return Math.abs(getMotorSpeed() - targetRpm) <= FlywheelConstants.RPM_TOLERANCE;
  }

  public void toggle() {
    this.on = !this.on;
  }

  public Command spinFlywheelBackwards() {
    return startEnd(() -> setVoltage(Volts.of(4)), () -> spinDownCommand());
  }

  public Command spinUpCommand() {
    return new InstantCommand(() -> setState(true));
  }

  public Command spinDownCommand() {
    return new InstantCommand(() -> setState(false));
  }

  private double getRpmForDistance(double distance) {
    var map = FlywheelConstants.DISTANCE_TO_RPM_MAP;
    if (map.containsKey(distance)) {
      return map.get(distance);
    }

    var lower = map.floorEntry(distance);
    var higher = map.ceilingEntry(distance);

    if (lower == null && higher == null) {
      return FlywheelConstants.LOWEST_RPM;
    } else if (lower == null) {
      return higher.getValue();
    } else if (higher == null) {
      return lower.getValue();
    } else {
      double x0 = lower.getKey();
      double y0 = lower.getValue();
      double x1 = higher.getKey();
      double y1 = higher.getValue();
      double t = (distance - x0) / (x1 - x0);
      return y0 + t * (y1 - y0);
    }
  }

  @Override
  public void periodic() {
    double targetRpm;
    if (this.on) {
      targetRpm = getRpmForDistance(distanceToHub.getAsDouble());
    } else {
      targetRpm = 0;
    }
    double output = 0;
    if (targetRpm > 0.001 || targetRpm < -0.001) {
      output += feedforward.calculate(targetRpm / 60);
      output += pid.calculate(getMotorSpeed(), targetRpm);
    } else {
      output = 0;
    }

    setVoltage(Volts.of(output));

    SmartDashboard.putNumber(
        "Flywheel/ Applied Output",
        rightFlywheelMotor.getAppliedOutput() * rightFlywheelMotor.getBusVoltage());
    SmartDashboard.putNumber("Flywheel/ Speed", getMotorSpeed());
    SmartDashboard.putNumber("Flywheel/ Output", output);
    SmartDashboard.putNumber("Flywheel/Target RPM", targetRpm);
    SmartDashboard.putBoolean("Flywheel/At Target Speed", isAtTargetSpeed());
  }
}
