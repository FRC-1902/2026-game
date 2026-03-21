// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Flywheel.FlywheelSubsystem;
import frc.robot.subsystems.Flywheel.FlywheelConstants;
import frc.robot.subsystems.Telemetry;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.io.File;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  public IntakeSubsystem intake;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController manipXbox = new CommandXboxController(1);

  private final VisionSubsystem vision = new VisionSubsystem();

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase =
      new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));

  public final Telemetry telemetry = new Telemetry();
  private final FlywheelSubsystem flywheel;
  private final IndexerSubsystem indexer;
  private final HoodSubsystem hood = new HoodSubsystem();

  // Establish a Sendable Chooser that will be able to be sent to the SmartDashboard, allowing
  // selection of desired auto
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular
   * velocity.
   */
  SwerveInputStream driveRobotOriented =
      SwerveInputStream.of(
              drivebase.getSwerveDrive(),
              () -> driverXbox.getLeftY() * -1,
              () -> driverXbox.getLeftX() * -1)
          .withControllerRotationAxis(this::getInvertedRightX)
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.8)
          .robotRelative(true);

  SwerveInputStream driveAngularVelocity =
      SwerveInputStream.of(
              drivebase.getSwerveDrive(),
              () -> driverXbox.getLeftY() * -1,
              () -> driverXbox.getLeftX() * -1)
          .withControllerRotationAxis(this::getInvertedRightX)
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    flywheel = new FlywheelSubsystem();
    telemetry.setDrivebase(drivebase);
    indexer = new IndexerSubsystem();
    intake = new IntakeSubsystem();
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    // Set the default auto (do nothing)
    autoChooser.setDefaultOption("Do Nothing", Commands.none());

    // Add a simple auto option to have the robot drive forward for 1 second then stop
    autoChooser.addOption("Drive Forward", drivebase.driveForward().withTimeout(1));

  autoChooser.addOption(
      "try to shoot preload",
      // Zero gyro, set hood angle, start flywheel target, wait until at target, then run indexer
      drivebase
          .runOnce(drivebase::zeroGyroWithAlliance)
          .andThen(
              // set hood to min angle
              hood.runOnce(
                      () -> hood.setAngle(Rotation2d.fromDegrees(HoodConstants.HOOD_MIN_ANGLE)))
                  // then set the flywheel target
                  .andThen(
                      flywheel.runOnce(
                          () -> flywheel.setTargetRpm(FlywheelConstants.DESIRED_FLYWHEEL_RPM))
                          // wait until flywheel reports at-target
                          .andThen(new WaitUntilCommand(flywheel::isAtTarget))
                          // then run the indexer while flywheel remains spinning
                          .andThen(indexer.spinRollerShooterCommand().withTimeout(10.0)))));

    // Put the autoChooser on the SmartDashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);

    // drivebase.setDefaultCommand(driveRobotOrientedAngularVelocity);

    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
  manipXbox.leftBumper().whileTrue(indexer.outtakeCommand());
    manipXbox
        .rightBumper()
        .onTrue(
            new FunctionalCommand(
                // init: toggle the flywheel
                () -> flywheel.toggle(),
                // then: turn on rumble
                () ->
                    manipXbox.setRumble(
                        edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble, 1.0),
                // end: stop rumble
                interrupted ->
                    manipXbox.setRumble(
                        edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble, 0.0),
                // isFinished: if flywheel is enabled, wait until it's at target; otherwise wait until stopped
                () -> {
                  if (flywheel.isEnabled()) {
                    return flywheel.isAtTarget();
                  } else {
                    return flywheel.isStopped();
                  }
                },
                // requirements: require the flywheel subsystem
                flywheel));
  manipXbox
    .povLeft()
    .onTrue(hood.runOnce(() -> hood.setAngle(Rotation2d.fromDegrees(HoodConstants.HOOD_SETPOINT_1))));
  manipXbox
    .povUp()
    .onTrue(hood.runOnce(() -> hood.setAngle(Rotation2d.fromDegrees(HoodConstants.HOOD_SETPOINT_2))));
  manipXbox
    .povRight()
    .onTrue(hood.runOnce(() -> hood.setAngle(Rotation2d.fromDegrees(HoodConstants.HOOD_SETPOINT_3))));
  manipXbox
    .povDown()
    .onTrue(hood.runOnce(() -> hood.setAngle(Rotation2d.fromDegrees(HoodConstants.HOOD_SETPOINT_4))));
    manipXbox.x().onTrue(intake.enableIntakeCommand());
    manipXbox.b().onTrue(intake.disableIntakeCommand());
    manipXbox.leftTrigger().whileTrue(intake.startRollersCommand());
    manipXbox
        .rightTrigger()
      .whileTrue(indexer.spinRollerShooterCommand().alongWith(intake.startRollersCommand()));
  driverXbox.x().onTrue(drivebase.runOnce(drivebase::zeroGyroWithAlliance));

  // Hold to shimmy
  driverXbox
      .leftBumper()
      .whileTrue(
          new FunctionalCommand(
              () -> {
                // No init
              },
              () -> {
                // Toggle direction every 0.1s: +speed for first half, -speed for second half.
                double periodSec = 0.2;
                double speed = 1.5; // m/s robot-relative forward/back
                double phase = Timer.getFPGATimestamp() % periodSec;
                double shimmySpeed = (phase < periodSec / 2.0) ? speed : -speed;
                drivebase.drive(new Translation2d(shimmySpeed, 0.0), 0.0, false);
              },
              interrupted -> drivebase.drive(new Translation2d(0.0, 0.0), 0.0, false),
              () ->
                  Math.abs(driverXbox.getLeftX()) > 0.01
                      || Math.abs(driverXbox.getLeftY()) > 0.01
                      || Math.abs(driverXbox.getRightX()) > 0.01
                      || Math.abs(driverXbox.getRightY()) > 0.01,
              drivebase));

  driverXbox.y().whileTrue(vision.processClosestTagCommand(hood));

  manipXbox.y().onTrue(hood.runOnce(() -> hood.setAngle(hood.upOneDegree())));
  manipXbox.a().onTrue(hood.runOnce(() -> hood.setAngle(hood.downOneDegree())));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Pass in the selected auto from the SmartDashboard as our desired autnomous commmand
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }

  public boolean flipForAlliance() {
    var alliance = DriverStation.getAlliance();
    Alliance ourAlliance = alliance.get();
    if (ourAlliance == Alliance.Red) {
      return true;
    } else {
      return false;
    }
  }

  public double getInvertedRightX() {
    return driverXbox.getRightX() * -1;
  }

public Command vibrateController(double intensity, double seconds) {
  return Commands.startEnd(
          () ->
              manipXbox.setRumble(
                  edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble, intensity),
          () ->
              manipXbox.setRumble(
                  edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble, 0))
      .withTimeout(seconds);
}
}