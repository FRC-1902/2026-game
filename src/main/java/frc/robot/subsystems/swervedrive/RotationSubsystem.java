package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.geometry.Rotation2d;

public class RotationSubsystem {

    Rotation2d desiredAngle;
    private final double rotationP = .08;
    
    public Rotation2d getCurrentDelta(Rotation2d currentRot) {
        return desiredAngle.minus(currentRot);
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
