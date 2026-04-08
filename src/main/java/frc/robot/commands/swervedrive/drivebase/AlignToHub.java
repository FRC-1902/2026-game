package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.systems.field.AllianceFlipUtil;
import frc.robot.systems.field.FieldConstants.Hub;
import swervelib.SwerveInputStream;

public class AlignToHub extends Command {
  private final SwerveSubsystem swerveSubsystem;
  private final SwerveInputStream swerveInputStream;

  public AlignToHub(SwerveSubsystem swerveSubsystem, SwerveInputStream swerveInputStream) {
    this.swerveSubsystem = swerveSubsystem;
    this.swerveInputStream = swerveInputStream.copy().aimLookahead(Units.Milliseconds.of(100));
    addRequirements(this.swerveSubsystem);
  }

  @Override
  public void initialize() {
    swerveInputStream
        .aim(
            AllianceFlipUtil.apply(
                new Pose2d(Hub.topCenterPoint.toTranslation2d(), Rotation2d.kZero)))
        .aimWhile(true)
        .scaleTranslation(0.7);
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
