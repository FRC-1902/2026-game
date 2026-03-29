package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.hood.AlignHoodCommand;
import frc.robot.commands.swervedrive.drivebase.AlignToHub;
import frc.robot.subsystems.Flywheel.FlywheelSubsystem;
import frc.robot.subsystems.WaypointManager;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.function.BooleanSupplier;

/** Aligns robot + hood, spins flywheel to interpolation target and rumbles when ready. */
public class PrepForShotCommand extends SequentialCommandGroup {

  public PrepForShotCommand(
      SwerveSubsystem drivebase,
      WaypointManager waypointManager,
      HoodSubsystem hood,
      FlywheelSubsystem flywheel,
      IndexerSubsystem indexer,
      CommandXboxController manipController,
      BooleanSupplier flipForAllianceSupplier) {

    Command readyRumble =
        Commands.startEnd(
                () -> manipController.setRumble(GenericHID.RumbleType.kBothRumble, 1.0),
                () -> manipController.setRumble(GenericHID.RumbleType.kBothRumble, 0.0))
            .withTimeout(1);

    addCommands(
        flywheel.spinUpCommand(),
        Commands.parallel(
            new AlignToHub(drivebase, waypointManager, "HUB", flipForAllianceSupplier),
            new AlignHoodCommand(hood, drivebase, waypointManager, flipForAllianceSupplier)),
        Commands.waitUntil(flywheel::isAtTargetSpeed),
        readyRumble);
  }
}
