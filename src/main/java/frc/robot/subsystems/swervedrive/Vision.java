// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.VisionConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import swervelib.SwerveDrive;

public class Vision {

  public static final AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  private final Camera[] cameras;

  public Vision() {
    cameras =
        new Camera[] {
          new Camera(
              VisionConstants.Cameras.FRONT_LEFT_NAME,
              VisionConstants.Cameras.FRONT_LEFT_ROBOT_TO_CAM,
              VisionConstants.SINGLE_TAG_STD_DEVS,
              VisionConstants.MULTI_TAG_STD_DEVS),
          new Camera(
              VisionConstants.Cameras.FRONT_RIGHT_NAME,
              VisionConstants.Cameras.FRONT_RIGHT_ROBOT_TO_CAM,
              VisionConstants.SINGLE_TAG_STD_DEVS,
              VisionConstants.MULTI_TAG_STD_DEVS),
          new Camera(
              VisionConstants.Cameras.BACK_LEFT_NAME,
              VisionConstants.Cameras.BACK_LEFT_ROBOT_TO_CAM,
              VisionConstants.SINGLE_TAG_STD_DEVS,
              VisionConstants.MULTI_TAG_STD_DEVS),
          new Camera(
              VisionConstants.Cameras.BACK_RIGHT_NAME,
              VisionConstants.Cameras.BACK_RIGHT_ROBOT_TO_CAM,
              VisionConstants.SINGLE_TAG_STD_DEVS,
              VisionConstants.MULTI_TAG_STD_DEVS)
        };
  }

  /**
   * Update the pose estimation inside of {@link SwerveDrive} with all camera measurements. This
   * should be called periodically (every robot loop).
   */
  public void updatePoseEstimation(SwerveDrive swerveDrive) {
    // Lists for logging all camera observations combined
    List<Pose3d> allRobotPoses = new ArrayList<>();
    List<Pose3d> allRobotPosesAccepted = new ArrayList<>();
    List<Pose3d> allRobotPosesRejected = new ArrayList<>();

    for (int i = 0; i < cameras.length; i++) {
      Camera camera = cameras[i];

      // Lists for logging individual camera observations
      List<Pose3d> cameraRobotPoses = new ArrayList<>();
      List<Pose3d> cameraRobotPosesAccepted = new ArrayList<>();
      List<Pose3d> cameraRobotPosesRejected = new ArrayList<>();
      List<Pose3d> tagPoses = new ArrayList<>();

      // Log camera connection status
      boolean isConnected = camera.camera.isConnected();
      Logger.recordOutput("Vision/Camera" + i + "/Connected", isConnected);

      Optional<EstimatedRobotPose> poseEst = camera.getEstimatedGlobalPose();
      if (poseEst.isPresent()) {
        var pose = poseEst.get();

        // Log the raw camera pose
        cameraRobotPoses.add(pose.estimatedPose);

        // Check if pose was accepted (has valid std devs)
        boolean isAccepted = camera.curStdDevs.get(0, 0) < Double.MAX_VALUE;

        if (isAccepted) {
          cameraRobotPosesAccepted.add(pose.estimatedPose);

          // Add vision measurement to the pose estimator with calculated standard deviations
          swerveDrive.addVisionMeasurement(
              pose.estimatedPose.toPose2d(), pose.timestampSeconds, camera.curStdDevs);

          // Log the AprilTag poses that were used
          for (var target : pose.targetsUsed) {
            var tagPose = fieldLayout.getTagPose(target.getFiducialId());
            if (tagPose.isPresent()) {
              tagPoses.add(tagPose.get());
            }
          }

          // Log standard deviations used
          Logger.recordOutput(
              "Vision/Camera" + i + "/StdDevs",
              new double[] {
                camera.curStdDevs.get(0, 0),
                camera.curStdDevs.get(1, 0),
                camera.curStdDevs.get(2, 0)
              });

          // Log number of tags and average distance
          Logger.recordOutput("Vision/Camera" + i + "/TagCount", pose.targetsUsed.size());
          Logger.recordOutput("Vision/Camera" + i + "/AvgDistance", camera.lastAvgDistance);
          Logger.recordOutput("Vision/Camera" + i + "/Timestamp", pose.timestampSeconds);

        } else {
          cameraRobotPosesRejected.add(pose.estimatedPose);
        }
      }

      // Log individual camera data
      Logger.recordOutput(
          "Vision/Camera" + i + "/RobotPoses", cameraRobotPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + i + "/RobotPosesAccepted",
          cameraRobotPosesAccepted.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + i + "/RobotPosesRejected",
          cameraRobotPosesRejected.toArray(new Pose3d[0]));
      Logger.recordOutput("Vision/Camera" + i + "/TagPoses", tagPoses.toArray(new Pose3d[0]));

      // Add to combined lists
      allRobotPoses.addAll(cameraRobotPoses);
      allRobotPosesAccepted.addAll(cameraRobotPosesAccepted);
      allRobotPosesRejected.addAll(cameraRobotPosesRejected);
    }

    // Log summary data (all cameras combined)
    Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));

    // Log the current robot pose from the pose estimator (fused gyro + vision)
    Logger.recordOutput("Vision/FusedRobotPose", swerveDrive.getPose());
  }

  private class Camera {

    public final PhotonCamera camera;

    public final PhotonPoseEstimator poseEstimator;

    private final Matrix<N3, N1> singleTagStdDevs;

    private final Matrix<N3, N1> multiTagStdDevs;

    public Matrix<N3, N1> curStdDevs;

    public double lastAvgDistance = 0.0;

    Camera(
        String name,
        Transform3d robotToCamTransform,
        Matrix<N3, N1> singleTagStdDevs,
        Matrix<N3, N1> multiTagStdDevs) {
      camera = new PhotonCamera(name);

      poseEstimator = new PhotonPoseEstimator(Vision.fieldLayout, robotToCamTransform);

      this.singleTagStdDevs = singleTagStdDevs;
      this.multiTagStdDevs = multiTagStdDevs;
      this.curStdDevs = singleTagStdDevs;
    }

    /**
     * Get the estimated global pose from this camera.
     *
     * @return Estimated robot pose, or empty if no valid estimate
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
      List<PhotonPipelineResult> results = camera.getAllUnreadResults();

      Optional<EstimatedRobotPose> estimatedPose = Optional.empty();

      for (var result : results) {
        if (!result.hasTargets()) {
          continue;
        }
        // Ambiguity check: require best target ambiguity below threshold
        PhotonTrackedTarget bestTarget = result.getBestTarget();
        if (bestTarget != null && bestTarget.getPoseAmbiguity() > 0.2) { // 0.2 threshold
          continue;
        }

        Optional<EstimatedRobotPose> poseOpt = poseEstimator.estimateCoprocMultiTagPose(result);
        if (poseOpt.isEmpty()) {
          poseOpt = poseEstimator.estimateLowestAmbiguityPose(result);
        }

        if (poseOpt.isEmpty()) {
          continue;
        }
        var pose = poseOpt.get().estimatedPose;
        double x = pose.getX();
        double y = pose.getY();
        double fieldLengthMeters = Vision.fieldLayout.getFieldLength();
        double fieldWidthMeters = Vision.fieldLayout.getFieldWidth();
        boolean inField = x >= 0 && x <= fieldLengthMeters && y >= 0 && y <= fieldWidthMeters;
        if (!inField) {
          continue;
        }

        // If passed all checks, accept this pose
        estimatedPose = poseOpt;
        updateEstimationStdDevs(estimatedPose, result.getTargets());
        break;
      }

      return estimatedPose;
    }

    /**
     * Calculate standard deviations dynamically based on number of tags and distance. This helps
     * the pose estimator trust better measurements more.
     *
     * @param estimatedPose The estimated pose
     * @param targets All targets visible in this camera frame
     */
    private void updateEstimationStdDevs(
        Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
      if (estimatedPose.isEmpty()) {
        curStdDevs = singleTagStdDevs;
        lastAvgDistance = 0.0;
        return;
      }

      var estStdDevs = singleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

      // Calculate average distance to all visible tags
      for (var tgt : targets) {
        var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
        if (tagPose.isEmpty()) {
          continue;
        }
        numTags++;
        avgDist +=
            tagPose
                .get()
                .toPose2d()
                .getTranslation()
                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
      }

      if (numTags == 0) {
        curStdDevs = singleTagStdDevs;
        lastAvgDistance = 0.0;
      } else {
        avgDist /= numTags;
        lastAvgDistance = avgDist;

        // Use multi-tag std devs if multiple tags visible (more accurate)
        if (numTags > 1) {
          estStdDevs = multiTagStdDevs;
        }

        // Reject single tag estimates that are too far away (> 4 meters)
        // TODO: Tune this distance threshold based on camera quality
        if (numTags == 1 && avgDist > 4) {
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
          // Scale std devs by distance squared (farther = less trust)
          // TODO: Tune this scaling factor (currently 1/30)
          estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        }
        curStdDevs = estStdDevs;
      }
    }
  }
}