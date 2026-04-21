package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.hood.AlignHoodCommand;
import frc.robot.commands.swervedrive.drivebase.AlignToHub;
import frc.robot.subsystems.Flywheel.FlywheelSubsystem;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.systems.shooting.ShotSolutionProvider;
import swervelib.SwerveInputStream;

/**
 * Autonomous shooting routine that calculates distance to the hub, aligns hood/flywheel setpoints,
 * then feeds once flywheel speed is ready.
 */
public class AutoShoot extends ParallelCommandGroup {

  public AutoShoot(
      SwerveSubsystem drivebase,
      ShotSolutionProvider shotProvider,
      HoodSubsystem hood,
      FlywheelSubsystem flywheel,
      IndexerSubsystem indexer,
      SwerveInputStream driveStream) {

    addCommands(
        new AlignToHub(drivebase, driveStream, shotProvider),
        new AlignHoodCommand(hood, shotProvider),
        Commands.startEnd(() -> flywheel.setState(true), () -> flywheel.setState(false), flywheel),
        indexer.spinRollerShooterCommand(() -> shotProvider.get().converged()));
  }
}
