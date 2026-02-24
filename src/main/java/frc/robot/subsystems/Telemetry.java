package frc.robot.subsystems;

// Telemetry subsystem for logging 2026 FRC game data and robot state.

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class Telemetry {
  private Alliance turnOrder = null;
  private SwerveSubsystem drivebase;
  private boolean turnOrderLogged = false;
  public boolean canSpinUp = false;
  public boolean canSpinDown = false;

  // Game period timing constants (in seconds, relative to start of teleop)
  private static final double TRANSITION_END = 10.0;
  private static final double SHIFT_1_END = 35.0; // 10 + 25
  private static final double SHIFT_2_END = 60.0; // 35 + 25
  private static final double SHIFT_3_END = 85.0; // 60 + 25
  private static final double SHIFT_4_END = 110.0; // 85 + 25
  private static final double ENDGAME_END = 140.0; // 110 + 30

  private static final double SPIN_WINDOW = 3.0;

  private double teleopStartTime = -1;

  public void setTurnOrder(Alliance order) {
    this.turnOrder = order;
    SmartDashboard.putString("Turn Order", order != null ? order.toString() : "No Data");
  }

  public boolean isTurnOrderLogged() {
    return turnOrderLogged;
  }

  public void setDrivebase(SwerveSubsystem drivebase) {
    this.drivebase = drivebase;
  }

  public void onTeleopInit() {
    teleopStartTime = Timer.getFPGATimestamp();
  }

  public void update() {
    SmartDashboard.putString("Turn Order", turnOrder != null ? turnOrder.toString() : "No Data");
    SmartDashboard.putBoolean("Turn Order Logged", turnOrderLogged);

    if (drivebase != null) {
      Pose2d pose = drivebase.getPose();
      SmartDashboard.putNumber("Estimated X (m)", pose.getX());
      SmartDashboard.putNumber("Estimated Y (m)", pose.getY());
      SmartDashboard.putNumber("Estimated Heading (deg)", pose.getRotation().getDegrees());
    }

    // Update period timing metrics and flywheel status if teleop has started and we have turn order
    if (teleopStartTime > 0 && turnOrderLogged && turnOrder != null) {
      updatePeriodMetrics();
    }
  }

  private void updatePeriodMetrics() {
    double elapsedTime = Timer.getFPGATimestamp() - teleopStartTime;

    // Determine current period and time remaining
    String currentPeriod;
    double timeToNextPeriod;
    String upcomingPeriod;

    if (elapsedTime < TRANSITION_END) {
      currentPeriod = "Transition (Free)";
      timeToNextPeriod = TRANSITION_END - elapsedTime;
      upcomingPeriod = "Shift 1";
    } else if (elapsedTime < SHIFT_1_END) {
      currentPeriod = "Shift 1";
      timeToNextPeriod = SHIFT_1_END - elapsedTime;
      upcomingPeriod = "Shift 2";
    } else if (elapsedTime < SHIFT_2_END) {
      currentPeriod = "Shift 2";
      timeToNextPeriod = SHIFT_2_END - elapsedTime;
      upcomingPeriod = "Shift 3";
    } else if (elapsedTime < SHIFT_3_END) {
      currentPeriod = "Shift 3";
      timeToNextPeriod = SHIFT_3_END - elapsedTime;
      upcomingPeriod = "Shift 4";
    } else if (elapsedTime < SHIFT_4_END) {
      currentPeriod = "Shift 4";
      timeToNextPeriod = SHIFT_4_END - elapsedTime;
      upcomingPeriod = "Endgame (Free)";
    } else if (elapsedTime < ENDGAME_END) {
      currentPeriod = "Endgame (Free)";
      timeToNextPeriod = ENDGAME_END - elapsedTime;
      upcomingPeriod = "Match End";
    } else {
      currentPeriod = "Match Over";
      timeToNextPeriod = 0;
      upcomingPeriod = "N/A";
    }

    // Determine if robot can score
    boolean canScore = canRobotScore(elapsedTime);
    updateSpinStatus(elapsedTime);

    SmartDashboard.putString("Current Period", currentPeriod);
    SmartDashboard.putNumber("Time to Next Period (s)", timeToNextPeriod);
    SmartDashboard.putString("Upcoming Period", upcomingPeriod);
    SmartDashboard.putBoolean("Can Score", canScore);
  }

  /**
   * Determines if the robot's alliance can currently score based on the period. Transition (0-10s):
   * Free for both Shift 1 (10-35s): Locked to alliance who did NOT win auto (lost auto = inactive
   * first) Shift 2 (35-60s): Switched (won auto = inactive first) Shift 3 (60-85s): Switched again
   * Shift 4 (85-110s): Switched again Endgame (110-140s): Free for both
   */
  private boolean canRobotScore(double elapsedTime) {
    // Get our alliance
    var alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      return false;
    }

    Alliance ourAlliance = alliance.get();

    // Transition and Endgame are free
    if (elapsedTime < TRANSITION_END || elapsedTime >= SHIFT_4_END) {
      return true;
    }

    boolean isInactivePeriod;

    if (elapsedTime < SHIFT_1_END) {
      isInactivePeriod = ourAlliance != turnOrder;
    } else if (elapsedTime < SHIFT_2_END) {
      isInactivePeriod = ourAlliance == turnOrder;
    } else if (elapsedTime < SHIFT_3_END) {
      isInactivePeriod = ourAlliance != turnOrder;
    } else {
      isInactivePeriod = ourAlliance == turnOrder;
    }

    return isInactivePeriod;
  }

  private void updateSpinStatus(double elapsedTime) {
    canSpinDown = false;
    canSpinUp = false;
    if(elapsedTime <= TRANSITION_END) { // Can spin during the Transition Period
      canSpinUp = true;
    } else if (!canRobotScore(elapsedTime) && elapsedTime > TRANSITION_END) { // Can stop spinning if we don't have the first shift
      canSpinDown = true;
    } else if (!canRobotScore(elapsedTime) && elapsedTime >= SHIFT_1_END - SPIN_WINDOW) { // Can spin up before shift 2
      canSpinUp = true;
    } else if (!canRobotScore(elapsedTime) && elapsedTime >= SHIFT_1_END + SPIN_WINDOW) { // Can spin down after shift 1
      canSpinDown = true;
    } else if (!canRobotScore(elapsedTime) && elapsedTime >= SHIFT_2_END - SPIN_WINDOW) { // Can spin up before shift 3
      canSpinUp = true;
    } else if (!canRobotScore(elapsedTime) && elapsedTime >= SHIFT_2_END + SPIN_WINDOW) { // Can spin down after shift 2
      canSpinDown = true;
    } else if (!canRobotScore(elapsedTime) && elapsedTime >= SHIFT_3_END - SPIN_WINDOW) { // Can spin up before shift 4
      canSpinUp = true;
    } else if (!canRobotScore(elapsedTime) && elapsedTime >= SHIFT_3_END + SPIN_WINDOW) { // Can spin down after shift 3
      canSpinDown = true;
    } else if (elapsedTime >= SHIFT_4_END - SPIN_WINDOW) { // Can spin up during endgame
      canSpinUp = true;
    }
  }

  public void checkAndLogTurnOrder() {
    if (turnOrderLogged) return;
    String gameData = DriverStation.getGameSpecificMessage();
    if (gameData.length() > 0) {
      switch (gameData.charAt(0)) {
        case 'B':
          setTurnOrder(Alliance.Blue);
          break;
        case 'R':
          setTurnOrder(Alliance.Red);
          break;
        default:
          setTurnOrder(null);
          break;
      }
      turnOrderLogged = true;
    } else {
      setTurnOrder(null);
    }
  }
}
