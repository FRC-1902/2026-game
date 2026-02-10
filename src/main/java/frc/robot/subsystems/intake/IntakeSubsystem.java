package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkAbsoluteEncoder;


public class IntakeSubsystem extends SubsystemBase {

    private final SparkMax rollerMotor;
    private final SparkMax pivotMotor;
    private final SparkAbsoluteEncoder pivotEncoder;

    PIDController pid = new PIDController(IntakeConstants.INTAKE_KP,IntakeConstants.INTAKE_KI, IntakeConstants.INTAKE_KD);

    public IntakeSubsystem() {

        pivotMotor = new SparkMax(IntakeConstants.PIVOTMOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        rollerMotor = new SparkMax(IntakeConstants.ROLLERMOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        pivotEncoder = pivotMotor.getAbsoluteEncoder();

        SparkMaxConfig config = new SparkMaxConfig();

        config.closedLoop.pid(IntakeConstants.INTAKE_KP,IntakeConstants.INTAKE_KI, IntakeConstants.INTAKE_KD);
        config.closedLoop.iZone(0);
        config.closedLoop.outputRange(-1.0, 1.0);
        pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    private double targetAngle = 0.0;
    private boolean intakeEnabled = false;

    private double calculateGravityFeedforward(double angleDegrees) {
        double angleRadians = Math.toRadians(angleDegrees);
        return IntakeConstants.INTAKE_KG * Math.cos(angleRadians);
    }

    public double getAngle() {
        return pivotEncoder.getPosition() * 360.0 / IntakeConstants.MOTOR_TO_INTAKE_RATIO;
    }

    public void setAngle(double angle) {
        double ffVolts = calculateGravityFeedforward(angle);
        pivotMotor
            .getClosedLoopController()
            .setSetpoint(angle * (IntakeConstants.MOTOR_TO_INTAKE_RATIO / 360.0), ControlType.kPosition, ClosedLoopSlot.kSlot0, ffVolts);
    }

    public double getAbsoluteAngle() {
        // Get position as fraction of rotation (0.0 to 1.0)
        double absolutePosition = pivotEncoder.getPosition();
        // Convert to degrees (0 to 360)
        double rawAngle = absolutePosition * 360.0;
        // Account for encoder gearing
        double hoodAngle = rawAngle / IntakeConstants.INTAKE_TO_ENCODER_RATIO;
        return hoodAngle - IntakeConstants.ENCODER_OFFSET;
    }



    public void intakeDown() {
        targetAngle = IntakeConstants.ENABLED_INTAKE_ANGLE;
        setAngle(targetAngle);
        intakeEnabled = true;
    }

    public void startRollers() {
        rollerMotor.set(1.0);
    }

    public void stopRollers() {
        rollerMotor.stopMotor();
    }
    

    public void periodic() {

        double measurement = getAbsoluteAngle() * 360.0;
        double setpoint = targetAngle * 360.0;
        double pidoutput = pid.calculate(measurement, setpoint);
        double ff = calculateGravityFeedforward(getAbsoluteAngle());
        double output = pidoutput + ff;
        output = Math.max(-1.0, Math.min(1.0, output));
        pivotMotor.set(output);

        SmartDashboard.putNumber("Intake/Current Pivot Angle", getAngle());
        SmartDashboard.putNumber("Intake/Target Pivot Angle", targetAngle);
        SmartDashboard.putBoolean("Intake/Intake Enabled", intakeEnabled);
    }
}
