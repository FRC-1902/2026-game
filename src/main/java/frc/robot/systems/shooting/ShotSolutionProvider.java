package frc.robot.systems.shooting;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Flywheel.FlywheelConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.systems.field.FieldConstants;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class ShotSolutionProvider {

  private final SwerveSubsystem drivebase;
  private ShootingCalculator.ShotSolution currentSolution;

  public final DoubleSupplier effectiveDistance;
  public final Supplier<Translation2d> virtualTarget;

  public ShotSolutionProvider(SwerveSubsystem drivebase) {
    this.drivebase = drivebase;
    this.currentSolution = computeSolution();
    this.effectiveDistance = () -> currentSolution.effectiveDistance();
    this.virtualTarget = () -> currentSolution.virtualTarget();
  }

  public void update() {
    currentSolution = computeSolution();
    publishDashboard();
  }

  public ShootingCalculator.ShotSolution get() {
    return currentSolution;
  }

  private ShootingCalculator.ShotSolution computeSolution() {
    return ShootingCalculator.calculate(
        drivebase.getPose().getTranslation(),
        drivebase.getFieldVelocity(),
        FieldConstants.Hub.getHubTranslation2d(),
        FlywheelConstants.DISTANCE_TO_TOF_MAP);
  }

  private void publishDashboard() {
    SmartDashboard.putNumber("Shot/TOF (s)", currentSolution.timeOfFlight());
    SmartDashboard.putNumber("Shot/Effective dist (m)", currentSolution.effectiveDistance());
    SmartDashboard.putNumber(
        "Shot/Real dist (m)",
        drivebase.getPose().getTranslation().getDistance(FieldConstants.Hub.getHubTranslation2d()));
    SmartDashboard.putBoolean("Shot/Converged", currentSolution.converged());
  }
}
