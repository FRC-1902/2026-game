package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.systems.field.AllianceFlipUtil;
import frc.robot.systems.field.FieldConstants;
import java.util.HashMap;
import java.util.Map;

/** Manages waypoints */
public class WaypointManager {

  public static final String HUB_WAYPOINT = "HUB";

  private final Map<String, Pose2d> waypoints = new HashMap<>();

  public WaypointManager() {
    initializeWaypoints();
  }

  // ASSUME BLUE ALLIANCE FOR ALL ADDED WAYPOINTS, FLIP USING getWaypoint(name, true)
  private void initializeWaypoints() {
    waypoints.put(HUB_WAYPOINT, new Pose2d(FieldConstants.Hub.BLUE_HUB_POSITION, Rotation2d.kZero));
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
    if (AllianceFlipUtil.shouldFlip()) {
      return AllianceFlipUtil.apply(waypoint);
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
  public Pose2d getWaypointWithOffset(String name, double offsetMeters, boolean flipForAlliance) {
    Pose2d waypoint = getWaypoint(name, flipForAlliance);
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
  public double getDistanceToWaypoint(
      Pose2d currentPose, String waypointName, boolean flipForAlliance) {
    Pose2d waypoint = getWaypoint(waypointName, flipForAlliance);
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
  public Rotation2d getAngleToWaypoint(
      Pose2d currentPose, String waypointName, boolean flipForAlliance) {
    Pose2d waypoint = getWaypoint(waypointName, flipForAlliance);
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
