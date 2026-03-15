package frc.robot.commands.hood;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WaypointManager;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AlignHoodCommand extends Command {

  private final HoodSubsystem hood;
  private final SwerveSubsystem swerve;
  private final WaypointManager waypointManager;
  private boolean flipForAlliance;

  public AlignHoodCommand(
      HoodSubsystem hood,
      SwerveSubsystem swerve,
      WaypointManager waypointManager,
      boolean flipForAlliance) {
    this.hood = hood;
    this.swerve = swerve;
    this.waypointManager = waypointManager;
    this.flipForAlliance = flipForAlliance;

    addRequirements(hood);
  }

  @Override
  public void initialize() {
    updateHoodAngle();
  }

  @Override
  public void execute() {
    updateHoodAngle();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return hood.atSetpoint();
  }

  private void updateHoodAngle() {
    double distance =
        waypointManager.getDistanceToWaypoint(swerve.getPose(), "HUB", flipForAlliance);

    if (distance < 0) {
      return;
    }

    hood.setAngleForDistance(distance);

    SmartDashboard.putNumber("Hood/Distance to Hub", distance);
  }
}
