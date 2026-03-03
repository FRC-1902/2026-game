package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.function.DoubleSupplier;

public class AlignToHub extends Command {

  private final SwerveSubsystem swerve;
  private final PIDController thetaController;

  private double omegaOut = 0.0;

  public AlignToHub(SwerveSubsystem swerve) {
    this.swerve = swerve;

    thetaController = new PIDController(4.5, 0.0, 0.2);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(Math.toRadians(2.0));
  }

  @Override
  public void execute() {

    Pose2d currentPose = swerve.getPose();

    // Use WaypointManager to get angle to hub
    Rotation2d desiredAngle = swerve.getWaypointManager().getAngleToWaypoint(currentPose, "HUB");

    if (desiredAngle == null) {
      omegaOut = 0.0;
      return;
    }

    double omega =
        thetaController.calculate(
            currentPose.getRotation().getRadians(), desiredAngle.getRadians());

    // Clamp to robot max angular velocity
    double maxOmega = swerve.getSwerveDrive().getMaximumChassisAngularVelocity();
    omegaOut = Math.max(-maxOmega, Math.min(omega, maxOmega));
  }

  public DoubleSupplier getOmegaSupplier() {
    return () -> omegaOut;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
