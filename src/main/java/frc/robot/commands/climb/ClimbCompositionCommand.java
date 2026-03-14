package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.swervedrive.drivebase.AlignForClimb;
import frc.robot.commands.swervedrive.drivebase.AlignForClimb.Side;
import frc.robot.commands.swervedrive.drivebase.DriveToClimb;
import frc.robot.subsystems.climb.*;
import frc.robot.subsystems.climb.ClimbSubsystem.State;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.swervedrive.*;

public class ClimbCompositionCommand extends SequentialCommandGroup {

  private final IntakeSubsystem intake;
  private final SwerveSubsystem swerve;
  private final ClimbSubsystem climber;
  private Side side;

  public ClimbCompositionCommand(
      IntakeSubsystem intakeSubsystem,
      SwerveSubsystem swerveSubsystem,
      ClimbSubsystem climbSubsystem,
      Side side) {
    intake = intakeSubsystem;
    swerve = swerveSubsystem;
    climber = climbSubsystem;
    this.side = side;

    addCommands(
        intake.disableIntakeCommand(),
        new ParallelCommandGroup(
            climber.setStateCommand(State.UP), new DriveToClimb(swerve, this.side)),
        new AlignForClimb(swerve, this.side),
        climber.setStateCommand(State.CLIMBING));
  }
}
