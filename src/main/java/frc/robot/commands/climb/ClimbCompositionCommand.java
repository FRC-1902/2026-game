package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.climb.*;
import frc.robot.commands.swervedrive.*;
import frc.robot.subsystems.swervedrive.*;
import frc.robot.subsystems.intake.*;

public class ClimbCompositionCommand extends Command {
    
    private final IntakeSubsystem intake;
    private final SwerveSubsystem swerve;
    private final Climb climber;
    private boolean finished = false;

    public ClimbCompositionCommand(IntakeSubsystem intakeSubsystem, SwerveSubsystem swerveSubsystem, Climb climbSubsystem) {
        intake = intakeSubsystem;
        swerve = swerveSubsystem;
        climber = climbSubsystem;
        
        addRequirements(intake, swerve, climber);
    }

@Override
  public void initialize() {
    new InstantCommand(() -> new SequentialCommandGroup(
        
    ));
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return finished; //change later
  }


}

