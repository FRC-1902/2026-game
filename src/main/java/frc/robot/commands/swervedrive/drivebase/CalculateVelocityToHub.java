package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

// Math stuff for AlignToHub
// Calculates the angular velocity needed to face the hub based on the robot's current pose and the
// hub's position
public class CalculateVelocityToHub {

  private final PIDController thetaController;

  // TODO: replace with real hub location
  private final Translation2d hubPosition;

  public CalculateVelocityToHub(Translation2d hubPosition) {
    this.hubPosition = hubPosition;

    thetaController = new PIDController(4.5, 0.0, 0.2);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(Math.toRadians(2.0));
  }

  public double calculateOmega(Pose2d robotPose) {
    Translation2d robotPos = robotPose.getTranslation();
    Rotation2d robotRot = robotPose.getRotation();

    Translation2d toHub = hubPosition.minus(robotPos);
    Rotation2d desiredAngle = toHub.getAngle();

    return thetaController.calculate(robotRot.getRadians(), desiredAngle.getRadians());
  }

  public boolean atSetpoint() {
    return thetaController.atSetpoint();
  }
}
