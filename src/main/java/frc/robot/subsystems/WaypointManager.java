package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.swervedrive.Vision;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

/** Manages waypoints */
public class WaypointManager {

  private final Map<String, Pose2d> waypoints = new HashMap<>();

  public WaypointManager() {
    initializeWaypoints();
  }

  // TODO: Check these waypoints, specifically the rotation2ds.
  // ASSUME BLUE ALLIANCE FOR ALL ADDED WAYPOINTS, FLIP USING getWaypoint(name, true)
  private void initializeWaypoints() {
    waypoints.put("HUB", new Pose2d(4.624, 4.035, Rotation2d.fromDegrees(180)));
    // Ladder waypoints are based on the DRIVER STATION PERSPECTIVE.
    waypoints.put("LADDER_LEFT", new Pose2d(1.069, 4.503, Rotation2d.fromDegrees(0)));
    waypoints.put("LADDER_RIGHT", new Pose2d(1.069, 2.972, Rotation2d.fromDegrees(180)));
  }

  /**
   * Get a waypoint by name
   *
   * @param name Waypoint name
   * @return Waypoint pose, or null if not found
   */
  public Pose2d getWaypoint(String name) {
    return waypoints.get(name);
  }

  /**
   * Get a waypoint with alliance flipping
   *
   * @param name Waypoint name
   * @param flipForAlliance If true, flip for red alliance
   * @return Waypoint pose, or null if not found
   */
  public Pose2d getWaypoint(String name, boolean flipForAlliance) {
    Pose2d waypoint = waypoints.get(name);
    if (waypoint == null || !flipForAlliance) {
      return waypoint;
    }
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      double fieldLength = Vision.fieldLayout.getFieldLength();
      double fieldWidth = Vision.fieldLayout.getFieldWidth();
      return new Pose2d(
          fieldLength - waypoint.getX(),
          fieldWidth - waypoint.getY(),
          waypoint.getRotation().plus(Rotation2d.fromDegrees(180)));
    }
    return waypoint;
  }

  /**
   * Get a waypoint with an offset applied
   *
   * @param name Waypoint name
   * @param offsetMeters Offset in meters
   * @return Waypoint pose with offset applied
   */
  public Pose2d getWaypointWithOffset(String name, double offsetMeters) {
    Pose2d waypoint = waypoints.get(name);
    if (waypoint == null) {
      return null;
    }

    Translation2d offset = new Translation2d(offsetMeters, waypoint.getRotation());
    return new Pose2d(waypoint.getTranslation().plus(offset), waypoint.getRotation());
  }

  /**
   * Calculate distance from current pose to waypoint
   *
   * @param currentPose Current robot pose
   * @param waypointName Waypoint name
   * @return Distance in meters, or -1 if waypoint not found
   */
  public double getDistanceToWaypoint(Pose2d currentPose, String waypointName) {
    Pose2d waypoint = waypoints.get(waypointName);
    if (waypoint == null) {
      return -1;
    }

    return currentPose.getTranslation().getDistance(waypoint.getTranslation());
  }

  /**
   * Calculate angle from current pose to waypoint
   *
   * @param currentPose Current robot pose
   * @param waypointName Waypoint name
   * @return Angle to waypoint, or null if waypoint not found
   */
  public Rotation2d getAngleToWaypoint(Pose2d currentPose, String waypointName) {
    Pose2d waypoint = waypoints.get(waypointName);
    if (waypoint == null) {
      return null;
    }

    Translation2d delta = waypoint.getTranslation().minus(currentPose.getTranslation());
    Rotation2d fieldBearing = new Rotation2d(delta.getX(), delta.getY());
    return fieldBearing.minus(currentPose.getRotation());
  }

  /**
   * Add or update a waypoint
   *
   * @param name Waypoint name
   * @param pose Waypoint pose
   */
  public void setWaypoint(String name, Pose2d pose) {
    waypoints.put(name, pose);
  }
}