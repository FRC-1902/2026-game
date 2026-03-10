package frc.robot.commands.flywheel;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.Flywheel.*;
import frc.robot.subsystems.Telemetry;

public class FlywheelCommand extends Command {

  private final FlywheelSubsystem flywheelSubsystem;
  private final Telemetry telemetry = new Telemetry();

  public FlywheelCommand(FlywheelSubsystem flywheelSubsystem) {
    this.flywheelSubsystem = flywheelSubsystem;

    addRequirements(flywheelSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    telemetry.update();
    if (telemetry.canSpinUp) {
      flywheelSubsystem.spinToHighSpeed();
    } else if (telemetry.canSpinDown) {
      flywheelSubsystem.spinToLowSpeed();
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
