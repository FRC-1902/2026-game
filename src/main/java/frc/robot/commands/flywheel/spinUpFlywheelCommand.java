package frc.robot.commands.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel.FlywheelSubsystem;

public class spinUpFlywheelCommand extends Command {

  private final FlywheelSubsystem flywheelSubsystem;

  public spinUpFlywheelCommand(FlywheelSubsystem flywheelSubsystem) {
    this.flywheelSubsystem = flywheelSubsystem;
    addRequirements(flywheelSubsystem);
  }

  @Override
  public void initialize() {
    flywheelSubsystem.spinUpToSpeed();
  }

  @Override
  public void execute() {
    // No additional execution logic needed since the subsystem handles the control loop
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) flywheelSubsystem.spinDownToZero();
  }

  @Override
  public boolean isFinished() {
    return flywheelSubsystem.isAtTargetSpeed();
  }
}
