// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.VisionConstants;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
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
    for (Camera camera : cameras) {
      Optional<EstimatedRobotPose> poseEst = camera.getEstimatedGlobalPose();
      if (poseEst.isPresent()) {
        var pose = poseEst.get();
        // Add vision measurement to the pose estimator with calculated standard deviations
        swerveDrive.addVisionMeasurement(
            pose.estimatedPose.toPose2d(), pose.timestampSeconds, camera.curStdDevs);
      }
    }
  }

  private class Camera {

    public final PhotonCamera camera;

    public final PhotonPoseEstimator poseEstimator;

    private final Matrix<N3, N1> singleTagStdDevs;

    private final Matrix<N3, N1> multiTagStdDevs;

    public Matrix<N3, N1> curStdDevs;

    Camera(
        String name,
        Transform3d robotToCamTransform,
        Matrix<N3, N1> singleTagStdDevs,
        Matrix<N3, N1> multiTagStdDevs) {
      camera = new PhotonCamera(name);

      // Use MULTI_TAG_PNP_ON_COPROCESSOR for best accuracy
      poseEstimator =
          new PhotonPoseEstimator(
              Vision.fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamTransform);
      // Fall back to LOWEST_AMBIGUITY if multi-tag fails
      poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

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

        Optional<EstimatedRobotPose> poseOpt = poseEstimator.update(result);
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
      } else {
        avgDist /= numTags;

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