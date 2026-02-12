package frc.robot.commands.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel.FlywheelSubsystem;

public class spinDownToLowSpeedCommand extends Command {

  private final FlywheelSubsystem flywheelSubsystem;

  public spinDownToLowSpeedCommand(FlywheelSubsystem flywheelSubsystem) {
    this.flywheelSubsystem = flywheelSubsystem;
    addRequirements(flywheelSubsystem);
  }

  @Override
  public void initialize() {
    flywheelSubsystem.spinDownToLowSpeed();
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) flywheelSubsystem.spinDownToZero();
  }

  @Override
  public boolean isFinished() {
    return flywheelSubsystem.isAtTargetSpeed(); //Uses same method as spinUpToSpeed since the targetRpm is set to low speed in the subsystem code
  }
}
