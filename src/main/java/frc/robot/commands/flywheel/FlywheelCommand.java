package frc.robot.commands.flywheel;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.Flywheel.*;

public class FlywheelCommand extends Command {

  private final FlywheelSubsystem flywheelSubsystem;

  public FlywheelCommand(FlywheelSubsystem flywheelSubsystem) {
    this.flywheelSubsystem = flywheelSubsystem;

    addRequirements(flywheelSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      flywheelSubsystem.spinDownToZero();
    }
  }

  @Override
  public boolean isFinished() {
    return false; // Always returns false because this command should always be running in the
    // background
  }
}
