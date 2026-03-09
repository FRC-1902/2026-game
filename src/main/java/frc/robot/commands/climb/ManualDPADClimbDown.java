package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.climb.*;
import frc.robot.subsystems.climb.Climb.State;

public class ManualDPADClimbDown extends SequentialCommandGroup {

  private final Climb climber;

  public ManualDPADClimbDown(Climb climbSubsystem) {
    climber = climbSubsystem;
    

    if (climber.getState() == State.UP) {
      addCommands(new InstantCommand(() -> climber.setState(State.CLIMBING)));
    } else if (climber.getState() == State.RELEASING) {
      addCommands(
          new SequentialCommandGroup(
              new InstantCommand(() -> climber.setState(State.DOWN)),
              new InstantCommand(() -> climber.setState(State.OFF))));
    }
  }
}
