package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.climb.*;
import frc.robot.subsystems.climb.Climb.State;

public class ManualDPADClimbDown extends SequentialCommandGroup {
  public ManualDPADClimbDown(Climb climbSubsystem) {
    addCommands(
        // TODO: this needs some more work
        // maybe gate entering climb vs going down to be 2 different things?
        // also, think about difference between going up and releasing the bar up
        climbSubsystem.setStateCommand(State.CLIMBING));
  }
}
