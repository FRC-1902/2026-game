package frc.robot.commands.flywheel;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.flywheel.spinDownToLowSpeedCommand;
import frc.robot.commands.flywheel.spinUpFlywheelCommand;
import frc.robot.subsystems.Flywheel.FlywheelSubsystem;
import frc.robot.subsystems.Telemetry;

public class flywheelCompositionCommand extends Command{
    
    private final spinUpFlywheelCommand spinUpCommand;
    private final spinDownToLowSpeedCommand lowSpeedCommand;
    private final FlywheelSubsystem flywheelSubsystem;
    private final Telemetry telemetry = new Telemetry();
    private boolean isOurTurnToShoot;

    public flywheelCompositionCommand(FlywheelSubsystem flywheelSubsystem) {
        this.spinUpCommand = new spinUpFlywheelCommand(flywheelSubsystem);
        this.lowSpeedCommand = new spinDownToLowSpeedCommand(flywheelSubsystem);

        this.flywheelSubsystem = flywheelSubsystem;
    }

    @Override
  public void initialize() {}

  @Override
  public void execute() {
    
  }

  @Override
  public void end(boolean interrupted) {
    if(interrupted) {
    flywheelSubsystem.spinDownToZero();
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

