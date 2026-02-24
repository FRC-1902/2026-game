package frc.robot.commands.flywheel;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Flywheel.*;
import frc.robot.subsystems.Telemetry;

public class flywheelCompositionCommand extends Command {

  private final FlywheelSubsystem flywheelSubsystem;
  private final Telemetry telemetry = new Telemetry();

  public flywheelCompositionCommand(FlywheelSubsystem flywheelSubsystem) {
    this.flywheelSubsystem = flywheelSubsystem;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    telemetry.update();
    if (telemetry.canSpinUp) {
      CommandScheduler.getInstance().schedule(new spinUpFlywheelCommand(flywheelSubsystem));
    } else if (telemetry.canSpinDown) {
      CommandScheduler.getInstance().schedule(new spinDownToLowSpeedCommand(flywheelSubsystem));
    }
  }

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
