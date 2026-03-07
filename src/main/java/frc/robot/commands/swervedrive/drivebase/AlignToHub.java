package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WaypointManager;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.function.DoubleSupplier;
import swervelib.math.SwerveMath;

public class AlignToHub extends Command {
  private final SwerveSubsystem swerve;
  private final WaypointManager waypointManager;
  private final String waypointName;
  private final DoubleSupplier translationX;
  private final DoubleSupplier translationY;

  // This will probably need to be tuned
  private final PIDController rotController = new PIDController(0.05, 0, 0.002);

  public AlignToHub(
      SwerveSubsystem swerve,
      WaypointManager waypointManager,
      String waypointName,
      DoubleSupplier translationX,
      DoubleSupplier translationY) {
    this.swerve = swerve;
    this.waypointManager = waypointManager;
    this.waypointName = waypointName;
    this.translationX = translationX;
    this.translationY = translationY;

    // Declare subsystem dependencies
    addRequirements(swerve);

    // Tells controller -180 and 180 degrees are the same point
    rotController.enableContinuousInput(-180, 180);

    // Set tolerance: Stop attempting to correct if within 2 degrees
    rotController.setTolerance(2.0);
  }

  @Override
  public void execute() {
    Pose2d currentPose = swerve.getPose();
    Rotation2d targetAngle = waypointManager.getAngleToWaypoint(currentPose, waypointName);

    // Check if the waypoint exists
    double rotationOutput = 0;
    if (targetAngle != null) {
      rotationOutput =
          rotController.calculate(currentPose.getRotation().getDegrees(), targetAngle.getDegrees());

      // If within 2 degree tolerance, snap the rotation output to 0
      if (rotController.atSetpoint()) {
        rotationOutput = 0;
      }
    }

    double maxVelocity = swerve.getSwerveDrive().getMaximumChassisVelocity();
    Translation2d translation =
        SwerveMath.scaleTranslation(
            new Translation2d(
                translationX.getAsDouble() * maxVelocity, translationY.getAsDouble() * maxVelocity),
            0.8);

    swerve.drive(
        translation, rotationOutput, true // Field-centric
        );
  }

  @Override
  public void end(boolean interrupted) {
    return;
  }
}
