package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.hood.AlignHoodCommand;
import frc.robot.subsystems.Flywheel.FlywheelSubsystem;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.systems.shooting.ShotSolutionProvider;
import swervelib.SwerveInputStream;

public class AutoShoot extends ParallelCommandGroup {

  public AutoShoot(
      SwerveSubsystem drivebase,
      ShotSolutionProvider shotProvider,
      HoodSubsystem hood,
      FlywheelSubsystem flywheel,
      IndexerSubsystem indexer,
      SwerveInputStream driveStream) {

    addCommands(
        new AlignHoodCommand(hood, shotProvider),
        Commands.startEnd(() -> flywheel.setState(true), () -> flywheel.setState(false), flywheel),
        Commands.waitUntil(() -> flywheel.isAtTargetSpeed()),
        indexer.spinRollerShooterCommand());
  }
}
