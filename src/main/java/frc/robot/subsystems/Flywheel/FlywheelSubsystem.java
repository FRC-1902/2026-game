package frc.robot.subsystems.Flywheel;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.hood.HoodConstants;
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

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutAngularVelocity m_velocity = RadiansPerSecond.mutable(0);
  private final MutAngle m_position = Radians.mutable(0);

  private final SysIdRoutine m_sysIdRoutine;

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

    m_sysIdRoutine =
        new SysIdRoutine(
            // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
            new SysIdRoutine.Config(Volts.of(1).per(Second), Volts.of(7), Second.of(15)),
            new SysIdRoutine.Mechanism(
                // Tell SysId how to plumb the driving voltage to the motor(s).
                this::setVoltage,
                // Tell SysId how to record a frame of data for each motor on the mechanism being
                // characterized.
                log -> {
                  // Record a frame for the shooter motor.
                  log.motor("shooter-wheel")
                      .voltage(
                          m_appliedVoltage.mut_replace(
                              rightFlywheelMotor.getAppliedOutput()
                                  * rightFlywheelMotor.getBusVoltage(),
                              Volts))
                      .angularVelocity(
                          m_velocity.mut_replace(getMotorSpeed() / 60, RotationsPerSecond))
                      .angularPosition(
                          m_position.mut_replace(
                              rightFlywheelMotor.getEncoder().getPosition(), Rotations));
                },
                // Tell SysId to make generated commands require this subsystem, suffix test state
                // in
                // WPILog with this subsystem's name ("shooter")
                this));
  }

  private void setVoltage(Voltage v) {
    leftFlywheelMotor.setVoltage(v);
    rightFlywheelMotor.setVoltage(v);
  }

  public void turnOn(boolean on) {
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

  public Command spinUpCommand() {
    return new InstantCommand(); // TODO: make command that spins up and waits until you hit target
    // speed until it ends
  }

  public Command spinDownCommand() {
    return new InstantCommand(); // TODO: make command that spins up and waits until you hit target
    // speed until it ends
  }

  /**
   * Returns a command that will execute a quasistatic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  /**
   * Returns a command that will execute a dynamic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  private double getRpmForDistance(double distance) {
    var map = FlywheelConstants.DISTANCE_TO_RPM_MAP;
    if (map.containsKey(distance)) {
      return map.get(distance);
    }

    var lower = map.floorEntry(distance);
    var higher = map.ceilingEntry(distance);

    if (lower == null && higher == null) {
      return HoodConstants.HOOD_MIN_ANGLE;
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
    // This method will be called once per scheduler run
    SmartDashboard.putNumber(
        "Flywheel/ Applied Output",
        rightFlywheelMotor.getAppliedOutput() * rightFlywheelMotor.getBusVoltage());
    SmartDashboard.putNumber("Flywheel/ Speed", getMotorSpeed());
    SmartDashboard.putNumber("Flywheel/ Output", output);
    SmartDashboard.putNumber("Flywheel/Target RPM", FlywheelConstants.DESIRED_FLYWHEEL_RPM);
    SmartDashboard.putBoolean("Flywheel/At Target Speed", isAtTargetSpeed());
  }
}
