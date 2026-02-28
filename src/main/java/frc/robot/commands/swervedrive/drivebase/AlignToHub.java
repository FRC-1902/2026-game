package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.util.function.DoubleSupplier;
/* 
Turns towards a specific point (the hub),
uses a PID controller to calculate the necessary angular velocity (omega) 
to face the hub based on the robot's current pose and the hub's position
*/  

public class AlignToHub extends Command {

  private final SwerveSubsystem swerve;
  private final CalculateVelocityToHub CalculateVelocityToHub;
  private double omegaOut = 0.0;

  // TODO: set real hub position
  private static final Translation2d HUB_POSITION =
      new Translation2d(0.0, 0.0);

  public AlignToHub(SwerveSubsystem swerve) {
    this.swerve = swerve;
    this.CalculateVelocityToHub = new CalculateVelocityToHub(HUB_POSITION);
  }

  @Override
  public void execute() {
    double omega = CalculateVelocityToHub.calculateOmega(swerve.getPose());

    double maxOmega = swerve.getSwerveDrive().getMaximumChassisAngularVelocity();
    omegaOut = Math.max(-maxOmega, Math.min(omega, maxOmega));
  }

  public DoubleSupplier getOmegaSupplier() {
    return () -> omegaOut;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}