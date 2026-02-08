package frc.robot.subsystems.hood;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {

  private final SparkFlex hoodMotor;
  private final DutyCycleEncoder absoluteEncoder;

  private double targetAngle = 0.0;

  public Hood() {
    // Initialize motor
    hoodMotor = new SparkFlex(HoodConstants.HOOD_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

    // Configure motor
    SparkFlexConfig config = new SparkFlexConfig();

    // Set PID constants
    config.closedLoop.pid(HoodConstants.HOOD_KP, HoodConstants.HOOD_KI, HoodConstants.HOOD_KD);
    config.closedLoop.iZone(0);
    config.closedLoop.outputRange(-1.0, 1.0);

    config.encoder.positionConversionFactor(360.0 / HoodConstants.MOTOR_TO_HOOD_RATIO);

    // Apply configuration
    hoodMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Initialize absolute encoder
    absoluteEncoder = new DutyCycleEncoder(HoodConstants.HOOD_ENCODER_DIO_PORT);

    syncEncoders();
  }

  private void syncEncoders() {
    double absoluteAngle = getAbsoluteAngle();
    hoodMotor.getEncoder().setPosition(absoluteAngle * HoodConstants.MOTOR_TO_HOOD_RATIO / 360.0);
  }

  public double getAbsoluteAngle() {
    // Get position as fraction of rotation (0.0 to 1.0)
    double absolutePosition = absoluteEncoder.get();

    // Convert to degrees (0 to 360)
    double rawAngle = absolutePosition * 360.0;

    // Account for encoder gearing
    double hoodAngle = rawAngle / HoodConstants.HOOD_TO_ENCODER_RATIO;

    return hoodAngle - HoodConstants.ENCODER_OFFSET;
  }

  // Get the current hood angle from the motor encoder.
  public double getAngle() {
    return hoodMotor.getEncoder().getPosition() * (360.0 / HoodConstants.MOTOR_TO_HOOD_RATIO);
  }

  public double getTargetAngle() {
    return targetAngle;
  }

  private double calculateGravityFeedforward(double angleDegrees) {
    double angleRadians = Math.toRadians(angleDegrees);
    return HoodConstants.HOOD_KG * Math.cos(angleRadians);
  }

  public void setAngle(double angle) {
    // Clamp angle to safe limits
    targetAngle =
        Math.max(HoodConstants.HOOD_MIN_ANGLE, Math.min(HoodConstants.HOOD_MAX_ANGLE, angle));
    // Motor position is in degrees
    double ffVolts = calculateGravityFeedforward(targetAngle);
    hoodMotor
        .getClosedLoopController()
        .setSetpoint(targetAngle, ControlType.kPosition, ClosedLoopSlot.kSlot0, ffVolts);
  }

  // Get the interpolated hood angle for a given distance to target.

  public double getAngleForDistance(double distance) {
    return HoodConstants.DISTANCE_TO_ANGLE_MAP.get(distance);
  }

  // Set the hood angle based on distance to target using the interpolation table.

  public void setAngleForDistance(double distance) {
    double angle = getAngleForDistance(distance);
    setAngle(angle);
  }

  public boolean atTargetAngle() {
    return Math.abs(getAngle() - targetAngle) < HoodConstants.HOOD_ANGLE_TOLERANCE;
  }

  public void stop() {
    hoodMotor.stopMotor();
  }

  public void setPercentOutput(double percentOutput) {
    hoodMotor.set(percentOutput);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hood/Current Angle", getAngle());
    SmartDashboard.putNumber("Hood/Absolute Angle", getAbsoluteAngle());
    SmartDashboard.putNumber("Hood/Target Angle", targetAngle);
    SmartDashboard.putBoolean("Hood/At Target", atTargetAngle());
    SmartDashboard.putNumber("Hood/Motor Current", hoodMotor.getOutputCurrent());
    SmartDashboard.putNumber("Hood/Motor Output", hoodMotor.getAppliedOutput());
  }
}
