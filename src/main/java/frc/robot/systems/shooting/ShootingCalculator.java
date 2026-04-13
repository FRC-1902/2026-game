package frc.robot.systems.shooting;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.NavigableMap;

public final class ShootingCalculator {

  private static final int MAX_ITERATIONS = 5;
  private static final double TOF_TOLERANCE_SECONDS = 0.005;

  public record ShotSolution(
      Translation2d virtualTarget, double effectiveDistance, double timeOfFlight, boolean converged) {}

  private ShootingCalculator() {}

  public static ShotSolution calculate(
      Translation2d robotPosition,
      ChassisSpeeds fieldVelocity,
      Translation2d hubTranslation,
      NavigableMap<Double, Double> tofMap) {

    Translation2d virtualTarget = hubTranslation;
    double tof = 0.0;
    boolean converged = false;

    for (int i = 0; i < MAX_ITERATIONS; i++) {
      double distance = virtualTarget.getDistance(robotPosition);
      double nextTof = interpolate(distance, tofMap);

      virtualTarget =
          hubTranslation.minus(
              new Translation2d(
                  fieldVelocity.vxMetersPerSecond * nextTof,
                  fieldVelocity.vyMetersPerSecond * nextTof));

      if (Math.abs(nextTof - tof) < TOF_TOLERANCE_SECONDS) {
        tof = nextTof;
        converged = true;
        break;
      }

      tof = nextTof;
    }

    double effectiveDistance = virtualTarget.getDistance(robotPosition);
    return new ShotSolution(virtualTarget, effectiveDistance, tof, converged);
  }

  private static double interpolate(double x, NavigableMap<Double, Double> map) {
    if (map.containsKey(x)) {
      return map.get(x);
    }

    var lower = map.floorEntry(x);
    var upper = map.ceilingEntry(x);

    if (lower == null) {
      return upper.getValue();
    }
    if (upper == null) {
      return lower.getValue();
    }

    double x0 = lower.getKey();
    double y0 = lower.getValue();
    double x1 = upper.getKey();
    double y1 = upper.getValue();
    double t = (x - x0) / (x1 - x0);
    return y0 + t * (y1 - y0);
  }
}
