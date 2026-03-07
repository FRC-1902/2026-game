package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.swervedrive.drivebase.AlignForClimb;
import frc.robot.commands.swervedrive.drivebase.AlignForClimb.Side;
import frc.robot.commands.swervedrive.drivebase.PrepareToClimb;
import frc.robot.subsystems.climb.*;
import frc.robot.subsystems.climb.Climb.State;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.swervedrive.*;

public class ClimbCompositionCommand extends SequentialCommandGroup {

  private final IntakeSubsystem intake;
  private final SwerveSubsystem swerve;
  private final Climb climber;
  private Side side;

  public ClimbCompositionCommand(
      IntakeSubsystem intakeSubsystem,
      SwerveSubsystem swerveSubsystem,
      Climb climbSubsystem,
      Side side) {
    intake = intakeSubsystem;
    swerve = swerveSubsystem;
    climber = climbSubsystem;
    this.side = side;
    
    addCommands(
        intake.DisableIntakeCommand(),
        new InstantCommand(() -> new PrepareToClimb(swerve, this.side)),
        new InstantCommand(() -> climber.setState(State.UP), climber),
        new InstantCommand(() -> new AlignForClimb(swerve, climber.getSide())),
        new InstantCommand(() -> climber.setState(State.CLIMBING)));
  }
}
