package frc.robot.commands.hood;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.systems.shooting.ShotSolutionProvider;

public class AlignHoodCommand extends Command {

  private final HoodSubsystem hood;
  private final ShotSolutionProvider shotProvider;

  public AlignHoodCommand(HoodSubsystem hood, ShotSolutionProvider shotProvider) {
    this.hood = hood;
    this.shotProvider = shotProvider;

    addRequirements(hood);
  }

  @Override
  public void initialize() {
    updateHoodAngle();
  }

  @Override
  public void execute() {
    updateHoodAngle();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }

  private void updateHoodAngle() {
    double distance = shotProvider.effectiveDistance.getAsDouble();
    hood.setAngleForDistance(distance);
    SmartDashboard.putNumber("Hood/Distance (effective)", distance);
  }
}
