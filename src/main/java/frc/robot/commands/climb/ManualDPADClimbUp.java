package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.climb.*;
import frc.robot.subsystems.climb.Climb.State;

public class ManualDPADClimbUp extends SequentialCommandGroup {

  private final Climb climber;

  public ManualDPADClimbUp(Climb climbSubsystem) {
    climber = climbSubsystem;
    ;

    if (climber.getState() == State.OFF) {
      addCommands(new InstantCommand(() -> climber.setState(State.UP)));
    } else if (climber.getState() == State.CLIMBING) {
      addCommands(new InstantCommand(() -> climber.setState(State.RELEASING)));
    }

  }
}
