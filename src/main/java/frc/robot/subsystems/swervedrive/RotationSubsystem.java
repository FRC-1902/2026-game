package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RotationSubsystem {

    Rotation2d desiredAngle;
    private final double rotationP = .02;
    
    public Rotation2d getCurrentDelta(Rotation2d currentRot) {
        Rotation2d calculatedDelta = desiredAngle.minus(currentRot);

        SmartDashboard.putNumber("desiredAngle", desiredAngle.getDegrees());
        SmartDashboard.putNumber("currentRot", currentRot.getDegrees());
        SmartDashboard.putNumber("calculatedDelta", calculatedDelta.getDegrees());

        return calculatedDelta;
    }

    public double getCurrentDeltaScaled(Rotation2d currentRot) {
        Rotation2d delta = getCurrentDelta(currentRot);

        double out = delta.getDegrees() * rotationP;

        out = Math.min(Math.max(out, -1), 1);

        return out;
    }

    public void setDesiredAngle(Rotation2d desired) {
        desiredAngle = desired;
    }

    public void incrementDesiredAngle(double degrees) {
        desiredAngle = desiredAngle.plus(Rotation2d.fromDegrees(degrees));
    }
    
}
