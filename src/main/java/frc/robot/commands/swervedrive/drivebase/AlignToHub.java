package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.WaypointManager;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.function.BooleanSupplier;

public class AlignToHub extends Command {
  private final SwerveSubsystem swerve;
  private final WaypointManager waypointManager;
  private final BooleanSupplier flipForAllianceSupplier;

  public AlignToHub(
      SwerveSubsystem swerve,
      WaypointManager waypointManager,
      String waypointName,
      boolean flipForAlliance) {
    this(swerve, waypointManager, waypointName, () -> flipForAlliance);
  }

  public AlignToHub(
      SwerveSubsystem swerve,
      WaypointManager waypointManager,
      String waypointName,
      BooleanSupplier flipForAllianceSupplier) {
    this.swerve = swerve;
    this.waypointManager = waypointManager;
    this.flipForAllianceSupplier = flipForAllianceSupplier;
  }

  @Override
  public void execute() {
    Pose2d currentPose = swerve.getPose();

    Rotation2d targetDelta =
        waypointManager.getAngleToHub(currentPose, flipForAllianceSupplier.getAsBoolean()); // Delta

    Rotation2d desiredAbsoluteAngle = currentPose.getRotation().plus(targetDelta);

    // Check if the waypoint exists
    double rotationOutput = 0;
    if (targetDelta != null) {
      Robot.getInstance().m_robotContainer.rotSubsystem.setDesiredAngle(desiredAbsoluteAngle);
    }

    SmartDashboard.putNumber("Waypoints/Align To Hub Target Angle", targetDelta.getDegrees());
    SmartDashboard.putNumber(
        "Waypoints/Align To Hub Current Angle", currentPose.getRotation().getDegrees());
    SmartDashboard.putNumber("Waypoints/Align To Hub Output", rotationOutput);
  }

  @Override
  public void end(boolean interrupted) {
    return;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
