package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.hood.AlignHoodCommand;
import frc.robot.commands.swervedrive.drivebase.AlignToHub;
import frc.robot.subsystems.Flywheel.FlywheelSubsystem;
import frc.robot.subsystems.WaypointManager;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

/** Aligns robot + hood, spins flywheel to interpolation target and rumbles when ready. */
public class PrepForShotCommand extends ParallelCommandGroup {

  public PrepForShotCommand(
      SwerveSubsystem drivebase,
      WaypointManager waypointManager,
      HoodSubsystem hood,
      FlywheelSubsystem flywheel,
      SwerveInputStream driveStream) {

    addCommands(
        flywheel.spinUpCommand(),
        new AlignToHub(drivebase, driveStream),
        new AlignHoodCommand(hood, drivebase, waypointManager));
  }
}
