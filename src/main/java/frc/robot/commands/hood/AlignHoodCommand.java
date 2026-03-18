package frc.robot.commands.hood;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.hood.HoodSubsystem;

public class AlignHoodCommand extends Command {

  private final HoodSubsystem hood;
  private final Rotation2d target;

  public AlignHoodCommand(HoodSubsystem hood, Rotation2d target) {
    this.hood = hood;
    this.target = target;

    addRequirements(hood);
  }

  @Override
  public void initialize() {
    hood.setAngle(target);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return hood.atSetpoint();
  }
}
