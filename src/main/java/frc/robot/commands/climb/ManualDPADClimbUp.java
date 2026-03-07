package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.climb.*;
import frc.robot.subsystems.climb.Climb.State;

public class ManualDPADClimbUp extends SequentialCommandGroup {

  private final Climb climber;
  private boolean climbBool = false;

  public ManualDPADClimbUp(Climb climbSubsystem) {
    climber = climbSubsystem;
    ;

    if (climbBool) {
      addCommands(new InstantCommand(() -> climber.setState(State.UP)));
    } else if (!climbBool) {
      addCommands(new InstantCommand(() -> climber.setState(State.RELEASING)));
    }

    climbBool = !climbBool;
  }
}
