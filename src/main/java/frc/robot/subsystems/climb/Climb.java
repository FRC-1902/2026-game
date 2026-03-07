package frc.robot.subsystems.climb;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.swervedrive.drivebase.AlignForClimb;
import frc.robot.commands.swervedrive.drivebase.AlignForClimb.Side;
import java.util.Optional;

public class Climb extends SubsystemBase {
  public enum State {
    DOWN, // Moving towards the bottom-most position
    UP, // Moving position towards the top-most position
    CLIMBING, // Actively climbing up
    RELEASING, // Actively releasing down
    OFF // Not applying any power (use after DOWN state when it's at the bottom)
  }

  private final DigitalInput limitSwitch = new DigitalInput(ClimbConstants.LIMIT_SWITCH_PORT);
  private final SparkMax climbMotor;
  private final RelativeEncoder encoder;
  private double targetPosition;
  private State state;
  private double output;
  private Side side;

  // Set PID
  private final PIDController pid =
      new PIDController(ClimbConstants.CLIMB_KP, ClimbConstants.CLIMB_KI, ClimbConstants.CLIMB_KD);

  // Initialize motor and inbuilt encoder
  public Climb() {
    climbMotor = new SparkMax(ClimbConstants.CLIMB_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(SparkBaseConfig.IdleMode.kBrake);
    climbMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder = climbMotor.getEncoder();

    pid.setIZone(ClimbConstants.CLIMB_IZONE);
    pid.setTolerance(ClimbConstants.CLIMB_TOLERANCE);
  }

  // Set the desired state of the climb subsystem
  public void setState(State newState) {
    state = newState;
  }

  public void setSide(AlignForClimb.Side side2) {
    this.side = side2;
  }

  public AlignForClimb.Side getSide() {

    Optional<Alliance> alliance = DriverStation.getAlliance();
    if ((alliance.isPresent() && alliance.get() == Alliance.Red) && side == Side.LEFT) {
      side = Side.RIGHT;
    } else if ((alliance.isPresent() && alliance.get() == Alliance.Red) && side == Side.RIGHT) {
      side = Side.LEFT;
    }

    return side;
  }

  // Set the desired target position for the climb subsystem
  private void setTargetPosition(double position) {
    targetPosition = position;
  }

  // Check if the PID controller is at the setpoint
  public boolean atSetpoint() {
    return pid.atSetpoint();
  }

  // Check if the limit switch is triggered
  public boolean isLimitSwitchTriggered() {
    return limitSwitch.get();
  }

  // Get the current position of the climb subsystem based on the internal encoder
  private double getClimbPosition() {
    return encoder.getPosition();
  }

  // Calculate the gravity feedforward based on the current state
  private double calculateGravityFeedforward() {
    switch (state) {
      case RELEASING:
        return ClimbConstants.CLIMB_RELEASING_KCOS;
      case DOWN:
      case UP:
        return ClimbConstants.CLIMB_HOLDING_KCOS;
      default:
        return 0.0;
    }
  }

  private double calculateClimbTargetPosition() {
    switch (state) {
      case DOWN:
      case CLIMBING:
      case OFF:
        setTargetPosition(ClimbConstants.CLIMB_MIN_HEIGHT);
        return ClimbConstants.CLIMB_MIN_HEIGHT;
      case UP:
      case RELEASING:
        setTargetPosition(ClimbConstants.CLIMB_MAX_HEIGHT);
        return ClimbConstants.CLIMB_MAX_HEIGHT;
      default:
        setTargetPosition(ClimbConstants.CLIMB_MIN_HEIGHT);
        return ClimbConstants.CLIMB_MIN_HEIGHT;
    }
  }

  @Override
  public void periodic() {
    // state machine for controlling the climb motor based on the current state and target position
    switch (state) {
        // For DOWN and UP states, use PID control to maintain the target position with gravity
        // feedforward
      case DOWN:
      case UP:
        setTargetPosition(calculateClimbTargetPosition());
        double measurement = getClimbPosition();
        double pidOutput = pid.calculate(measurement, targetPosition);
        double ff = calculateGravityFeedforward();
        output = pidOutput + ff;
        output = Math.max(-1.0, Math.min(1.0, output));
        climbMotor.set(output);
        break;

        // For RELEASING state, use PID control to move towards the target position with a stronger
        // gravity feedforward to assist in releasing downwards
      case RELEASING:
        setTargetPosition(calculateClimbTargetPosition());
        measurement = getClimbPosition();
        pidOutput = pid.calculate(measurement, targetPosition);
        ff = calculateGravityFeedforward();
        output = pidOutput + ff;
        output = Math.max(-1.0, Math.min(1.0, output));
        climbMotor.set(output);
        break;

        // For CLIMBING state, move the motor upwards unless the limit switch is triggered or the
        // climb position is below the minimum height
      case CLIMBING:
        setTargetPosition(calculateClimbTargetPosition());
        if (!isLimitSwitchTriggered() && getClimbPosition() > ClimbConstants.CLIMB_MIN_HEIGHT) {
          climbMotor.set(ClimbConstants.CLIMB_CLIMBING_SPEED);
        } else {
          climbMotor.set(0.0);
        }
        break;

        // For OFF state, stop the motor and do not apply any power
      case OFF:
      default:
        climbMotor.set(0.0);
        break;
    }

    SmartDashboard.putNumber("Climb Position", getClimbPosition());
    SmartDashboard.putString("Climb State", state.toString());
    SmartDashboard.putNumber("Climb Output", output);
    SmartDashboard.putBoolean("Climb Limit Switch", isLimitSwitchTriggered());
  }
}
