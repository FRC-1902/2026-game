package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hood.HoodSubsystem;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {

  public static final String FRONT_LEFT_NAME = "arducamThree";
  public static final Transform3d FRONT_LEFT_ROBOT_TO_CAM =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-8.722503),
              Units.inchesToMeters(12.076808),
              Units.inchesToMeters(18.970713)),
          new Rotation3d(Math.toRadians(10), Math.toRadians(-22.67), Math.toRadians(25.696)));

  public static final String FRONT_RIGHT_NAME = "ArducamZero";
  public static final Transform3d FRONT_RIGHT_ROBOT_TO_CAM =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-4.697477),
              Units.inchesToMeters(-10.859125),
              Units.inchesToMeters(18.936709)),
          new Rotation3d(Math.toRadians(10), Math.toRadians(-19.4175), Math.toRadians(-18.195)));

  private final PhotonCamera frontLeft = new PhotonCamera(FRONT_LEFT_NAME);
  private final PhotonCamera frontRight = new PhotonCamera(FRONT_RIGHT_NAME);

  private static final double DEFAULT_TARGET_HEIGHT_METERS = 1.12395;

  public static class ClosestTarget {
    public final double distanceMeters;
    public final int fiducialId;
    public final String cameraName;

    public ClosestTarget(double distanceMeters, int fiducialId, String cameraName) {
      this.distanceMeters = distanceMeters;
      this.fiducialId = fiducialId;
      this.cameraName = cameraName;
    }
  }

  public VisionSubsystem() {}

  public Optional<ClosestTarget> getClosestTarget() {
    double closestDistance = Double.POSITIVE_INFINITY;
    int closestFiducial = -1;
    String closestCamera = "";

    List<PhotonPipelineResult> results = frontLeft.getAllUnreadResults();
    if (!results.isEmpty()) {
      var result = results.get(results.size() - 1);
      if (result.hasTargets()) {
        double camHeight = FRONT_LEFT_ROBOT_TO_CAM.getTranslation().getZ();
        double camPitch = FRONT_LEFT_ROBOT_TO_CAM.getRotation().getY(); // radians
        for (PhotonTrackedTarget t : result.getTargets()) {
          var maybeCameraToTarget = t.getBestCameraToTarget();
          double dist;
          if (maybeCameraToTarget != null) {
            dist = maybeCameraToTarget.getTranslation().getNorm();
          } else {
            dist =
                PhotonUtils.calculateDistanceToTargetMeters(
                    camHeight,
                    DEFAULT_TARGET_HEIGHT_METERS,
                    camPitch,
                    Math.toRadians(t.getPitch()));
          }
          if (dist < closestDistance) {
            closestDistance = dist;
            closestFiducial = t.getFiducialId();
            closestCamera = FRONT_LEFT_NAME;
          }
        }
      }
    }

    results = frontRight.getAllUnreadResults();
    if (!results.isEmpty()) {
      var result = results.get(results.size() - 1);
      if (result.hasTargets()) {
        double camHeight = FRONT_RIGHT_ROBOT_TO_CAM.getTranslation().getZ();
        double camPitch = FRONT_RIGHT_ROBOT_TO_CAM.getRotation().getY(); // radians
        for (PhotonTrackedTarget t : result.getTargets()) {
          var maybeCameraToTarget = t.getBestCameraToTarget();
          double dist;
          if (maybeCameraToTarget != null) {
            dist = maybeCameraToTarget.getTranslation().getNorm();
          } else {
            dist =
                PhotonUtils.calculateDistanceToTargetMeters(
                    camHeight,
                    DEFAULT_TARGET_HEIGHT_METERS,
                    camPitch,
                    Math.toRadians(t.getPitch()));
          }
          if (dist < closestDistance) {
            closestDistance = dist;
            closestFiducial = t.getFiducialId();
            closestCamera = FRONT_RIGHT_NAME;
          }
        }
      }
    }

    if (closestDistance != Double.POSITIVE_INFINITY) {
      return Optional.of(new ClosestTarget(closestDistance, closestFiducial, closestCamera));
    }
    return Optional.empty();
  }

  public void processClosestAprilTagAndSetHood(HoodSubsystem hood) {
    var opt = getClosestTarget();
    if (opt.isPresent()) {
      var t = opt.get();
      SmartDashboard.putNumber("Vision/ClosestDistance", t.distanceMeters);
      SmartDashboard.putNumber("Vision/ClosestFiducialId", t.fiducialId);
      SmartDashboard.putString("Vision/ClosestCamera", t.cameraName);
      System.out.printf(
          "Closest AprilTag %d from %s at %.2fm\n", t.fiducialId, t.cameraName, t.distanceMeters);
      hood.setAngleForDistance(t.distanceMeters);
    } else {
      SmartDashboard.putString("Vision/ClosestCamera", "none");
    }
  }

  public Command processClosestTagCommand(HoodSubsystem hood) {
    return Commands.run(() -> processClosestAprilTagAndSetHood(hood), this, hood);
  }
}
