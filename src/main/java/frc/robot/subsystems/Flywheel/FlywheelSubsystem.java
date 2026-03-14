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
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

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

    m_sysIdRoutine =
        new SysIdRoutine(
            // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
            new SysIdRoutine.Config(Volts.of(4).per(Second), Volts.of(4), Second.of(10)),
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
                              rightFlywheelMotor.get() * RobotController.getBatteryVoltage(),
                              Volts))
                      .angularPosition(
                          m_position.mut_replace(
                              rightFlywheelMotor.getEncoder().getPosition(), Rotations))
                      .angularVelocity(
                          m_velocity.mut_replace(getFlywheelSpeed(), RotationsPerSecond));
                },
                // Tell SysId to make generated commands require this subsystem, suffix test state
                // in
                // WPILog with this subsystem's name ("shooter")
                this));
  }

  // Create a new SysId routine for characterizing the flywheel.

  private void setVoltage(Voltage v) {
    leftFlywheelMotor.setVoltage(v);
    rightFlywheelMotor.setVoltage(v);
  }

  private void setSpeed(double s) {
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

  public Command testCommand() {
    return startEnd(() -> setSpeed(1), () -> setSpeed(0));
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
