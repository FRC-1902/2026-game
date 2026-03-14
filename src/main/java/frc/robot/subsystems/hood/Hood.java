package frc.robot.subsystems.hood;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {
  private final SparkFlex hoodMotor;
  private final DutyCycleEncoder absoluteEncoder;
  private Rotation2d newAngle;
  private double clamped;

  private Rotation2d targetAngle = new Rotation2d();
  private final PIDController pid =
      new PIDController(HoodConstants.HOOD_KP, HoodConstants.HOOD_KI, HoodConstants.HOOD_KD);

  public Hood() {
    // Initialize motor
    hoodMotor = new SparkFlex(HoodConstants.HOOD_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

    // Configure motor
    SparkFlexConfig config = new SparkFlexConfig();

    config.idleMode(SparkBaseConfig.IdleMode.kBrake);
    config.inverted(true);

    config.closedLoop.outputRange(-1.0, 1.0);
    config.encoder.positionConversionFactor(360.0 / HoodConstants.MOTOR_TO_HOOD_RATIO);

    hoodMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    absoluteEncoder = new DutyCycleEncoder(HoodConstants.HOOD_ENCODER_DIO_PORT);
    absoluteEncoder.setDutyCycleRange(1.0 / 1025.0, 1024.0 / 1025.0);

    pid.enableContinuousInput(0, 360);

    pid.setIZone(HoodConstants.HOOD_IZONE.getDegrees());
    pid.setTolerance(HoodConstants.HOOD_ANGLE_TOLERANCE.getDegrees());
  }

  public Rotation2d getAbsoluteAngle() {
    // Multiply by 360 to get encoder degrees, then divide by gear ratio to get hood degrees
    double encoderRotations = absoluteEncoder.get();
    double angle =
        (encoderRotations * 360.0 / HoodConstants.ENCODER_TO_HOOD_RATIO)
            - HoodConstants.ENCODER_OFFSET;

    // Wrap to 0-360 range
    angle = ((angle % 360.0) + 360.0) % 360.0;
    return Rotation2d.fromDegrees(angle);
  }

  // Get the current hood angle from the external encoder.

  public Rotation2d getTargetAngle() {
    return targetAngle;
  }

  private double calculateGravityFeedforward(Rotation2d angle) {
    return HoodConstants.HOOD_KCOS * Math.cos(angle.getRadians());
  }

  public void setAngle(Rotation2d angle) {
    // Clamp angle to valid range
    clamped =
        Math.max(
            HoodConstants.HOOD_MIN_ANGLE,
            Math.min(HoodConstants.HOOD_MAX_ANGLE, angle.getDegrees()));
    targetAngle = Rotation2d.fromDegrees(clamped);
  }

  // Get the interpolated hood angle for a given distance to target.
  private Rotation2d getAngleForDistance(double distance) {
    return Rotation2d.fromDegrees(HoodConstants.DISTANCE_TO_ANGLE_MAP.get(distance));
  }

  // Set the hood angle based on distance to target using the interpolation table.
  public void setAngleForDistance(double distance) {
    setAngle(getAngleForDistance(distance));
  }

  public boolean atTargetAngle() {
    return pid.atSetpoint();
  }

  public Rotation2d upOneDegree() {
    newAngle = Rotation2d.fromDegrees(clamped + 1);
    return newAngle;
  }

  public Rotation2d downOneDegree() {
    newAngle = Rotation2d.fromDegrees(clamped - 1);
    return newAngle;
  }

  @Override
  public void periodic() {
    // External PID control loop
    double measurement = getAbsoluteAngle().getDegrees();
    double setpoint = targetAngle.getDegrees();
    double pidOutput = pid.calculate(measurement, setpoint);
    double ff = calculateGravityFeedforward(getAbsoluteAngle());
    double output = pidOutput + ff;
    output = Math.max(-1.0, Math.min(1.0, output));
    hoodMotor.set(output);

    SmartDashboard.putNumber("Hood/Current Angle", measurement);
    SmartDashboard.putNumber("Hood/Absolute Angle", getAbsoluteAngle().getDegrees());
    SmartDashboard.putNumber("Hood/Target Angle", setpoint);
    SmartDashboard.putBoolean("Hood/At Target", atTargetAngle());
    SmartDashboard.putNumber("Hood/Motor Current", hoodMotor.getOutputCurrent());
    SmartDashboard.putNumber("Hood/Motor Output", hoodMotor.getAppliedOutput());
    SmartDashboard.putNumber("Hood/PID Output", pidOutput);
    SmartDashboard.putNumber("Hood/FF Output", ff);
  }
}
