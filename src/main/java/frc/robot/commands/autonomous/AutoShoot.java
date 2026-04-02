package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Flywheel.FlywheelSubsystem;
import frc.robot.subsystems.WaypointManager;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Autonomous shooting routine that calculates distance to the hub, aligns hood/flywheel setpoints,
 * then feeds once flywheel speed is ready.
 */
public class AutoShoot extends SequentialCommandGroup {

  public AutoShoot(
      SwerveSubsystem drivebase,
      WaypointManager waypointManager,
      HoodSubsystem hood,
      FlywheelSubsystem flywheel,
      IndexerSubsystem indexer,
      BooleanSupplier flipForAllianceSupplier) {

    DoubleSupplier distanceToHubSupplier =
        () ->
            waypointManager.getDistanceToWaypoint(
                drivebase.getPose(), "HUB", flipForAllianceSupplier.getAsBoolean());

    addCommands(
        Commands.runOnce(
            () -> {
              double distance = distanceToHubSupplier.getAsDouble();
              SmartDashboard.putNumber("Auto/Distance To Hub", distance);
            }),
        flywheel.spinUpCommand(),
        Commands.waitUntil(flywheel::isAtTargetSpeed),
        indexer.spinRollerShooterCommand(),
        flywheel.spinDownCommand());
  }
}
