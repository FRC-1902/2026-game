package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AutoShootToggleOnCommand extends InstantCommand {
  private final Command autoShoot;

  public AutoShootToggleOnCommand(Command autoShoot) {
    this.autoShoot = autoShoot;
  }

  @Override
  public void initialize() {
    if (!autoShoot.isScheduled()) {
      autoShoot.schedule();
    }
  }
}