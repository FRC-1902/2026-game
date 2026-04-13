package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.systems.shooting.ShotSolutionProvider;
import swervelib.SwerveInputStream;

public class AlignToHub extends Command {
  private final SwerveSubsystem swerveSubsystem;
  private final SwerveInputStream swerveInputStream;
  private final ShotSolutionProvider shotProvider;

  public AlignToHub(
      SwerveSubsystem swerveSubsystem,
      SwerveInputStream swerveInputStream,
      ShotSolutionProvider shotProvider) {
    this.swerveSubsystem = swerveSubsystem;
    this.swerveInputStream = swerveInputStream.copy();
    this.shotProvider = shotProvider;
    addRequirements(this.swerveSubsystem);
  }

  @Override
  public void initialize() {
    swerveInputStream
        .aim(() -> new Pose2d(shotProvider.virtualTarget.get(), Rotation2d.kZero))
        .aimWhile(true)
        .scaleTranslation(0.7)
        .aimLookahead(Units.Milliseconds.of(100));
  }

  @Override
  public void execute() {
    swerveSubsystem.driveFieldOrientedSetpoint(swerveInputStream.get());
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    swerveInputStream.aimWhile(false).scaleTranslation(1);
  }
}
