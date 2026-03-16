package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.hood.AlignHoodCommand;
import frc.robot.subsystems.WaypointManager;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class SimpleShootCommand extends ParallelCommandGroup {

  public SimpleShootCommand(
      SwerveSubsystem swerve,
      WaypointManager waypointManager,
      String waypointName,
      boolean flipForAlliance,
      HoodSubsystem hood) {
    addCommands(
        new AlignToHub(swerve, waypointManager, "HUB", flipForAlliance),
        new AlignHoodCommand(hood, swerve, waypointManager));
  }
}
