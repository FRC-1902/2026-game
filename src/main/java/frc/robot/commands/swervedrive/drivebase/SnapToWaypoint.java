package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.function.Supplier;

public class SnapToWaypoint extends Command {

  private final SwerveSubsystem swerve;
  private final Supplier<Pose2d> targetPoseSupplier;
  protected Pose2d targetPose;

  private final PIDController pidX;
  private final PIDController pidY;

  private final double maxVelocity;

  private final double distanceErrorTolerance = 0.06;

  private final double rotationErrorTolerance = Math.toRadians(3);

  protected double currentDistance;
  protected double currentRotError;

  public SnapToWaypoint(SwerveSubsystem swerve, Supplier<Pose2d> targetPoseSupplier) {
    this(swerve, targetPoseSupplier, 4.0);
  }

  public SnapToWaypoint(
      SwerveSubsystem swerve, Supplier<Pose2d> targetPoseSupplier, double maxVelocity) {
    this.swerve = swerve;
    this.targetPoseSupplier = targetPoseSupplier;
    this.maxVelocity = maxVelocity;

    // TODO: These values are from last years code, we need to TUNE them for this year (exciting,
    // isnt it?)
    this.pidX = new PIDController(2.5, 0.02, 0.001);
    this.pidY = new PIDController(2.5, 0.02, 0.001);

    pidX.reset();
    pidY.reset();

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    targetPose = targetPoseSupplier.get();
    pidX.reset();
    pidY.reset();
  }

  @Override
  public void execute() {
    Pose2d currentPose = swerve.getPose();

    currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
    currentRotError =
        Math.abs(targetPose.getRotation().minus(currentPose.getRotation()).getRadians());

    SmartDashboard.putNumber("Snap/Distance", currentDistance);
    SmartDashboard.putNumber("Snap/RotError", Math.toDegrees(currentRotError));

    double xVelocity = pidX.calculate(currentPose.getX(), targetPose.getX());
    double yVelocity = pidY.calculate(currentPose.getY(), targetPose.getY());

    Translation2d velocity = new Translation2d(xVelocity, yVelocity);
    Translation2d cappedVelocity = velocity;

    double v = velocity.getDistance(Translation2d.kZero);
    if (v > maxVelocity) {
      cappedVelocity = cappedVelocity.times(1.0 / v).times(maxVelocity);
    }

    // TODO: I think we need to tune the rotation P too.
    double rotationkP = 4.0;
    Rotation2d rotation =
        targetPose.getRotation().minus(currentPose.getRotation()).times(rotationkP);
    double cappedRotation = Math.max(Math.min(rotation.getRadians(), 3.0), -3.0);

    swerve.drive(cappedVelocity, cappedRotation, true);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(0, 0), 0, true);
  }

  @Override
  public boolean isFinished() {
    boolean positionReached = currentDistance < distanceErrorTolerance;
    boolean rotationReached = currentRotError < rotationErrorTolerance;
    return positionReached && rotationReached;
  }
}
