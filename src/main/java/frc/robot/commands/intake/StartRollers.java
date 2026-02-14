package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class StartRollers extends Command {

  private final IntakeSubsystem intake = new IntakeSubsystem();

  public void execute() {
    intake.startRollers();
  }

  public boolean isFinished() {
    return true;
  }

  public void end() {
    intake.stopRollers();
  }
}
