package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.function.Supplier;

public class DriveToClimb extends SequentialCommandGroup {
  public enum Side {
    LEFT,
    RIGHT
  }

  public DriveToClimb(SwerveSubsystem swerve, AlignForClimb.Side side) {
    String waypointName =
        side == AlignForClimb.Side.LEFT ? "LADDER_LEFT_PREPARE" : "LADDER_RIGHT_PREPARE";

    Supplier<Pose2d> targetPoseSupplier =
        () -> {
          Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
          SmartDashboard.putString("Climb/TargetSide", waypointName);
          SmartDashboard.putString("Climb/Alliance", alliance.name());
          return swerve.getWaypointManager().getWaypoint(waypointName, true);
        };

    addCommands(new SnapToWaypoint(swerve, targetPoseSupplier));
  }
}
