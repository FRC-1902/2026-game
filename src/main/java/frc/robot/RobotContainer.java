// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.flywheel.FlywheelCommand;
import frc.robot.commands.swervedrive.drivebase.AlignForClimb.Side;
import frc.robot.commands.swervedrive.drivebase.AlignToHub;
import frc.robot.commands.hood.AlignHoodCommand;
import frc.robot.subsystems.Flywheel.FlywheelSubsystem;
import frc.robot.subsystems.Telemetry;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.Climb.State;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;
import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  public IntakeSubsystem InstanceIntakeSubsystem = new IntakeSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController manipXbox = new CommandXboxController(1);

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase =
      new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));

  private final Vision vision = new Vision();
  public final Telemetry telemetry = new Telemetry();
  private final FlywheelSubsystem flywheel;
  private final Climb climber;
  private final IndexerSubsystem indexer;
  private final Hood hood;

  // Establish a Sendable Chooser that will be able to be sent to the SmartDashboard, allowing
  // selection of desired auto
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  private final SendableChooser<Side> climbSideChooser = new SendableChooser<>();

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular
   * velocity.
   */
  SwerveInputStream driveAngularVelocity =
      SwerveInputStream.of(
              drivebase.getSwerveDrive(),
              () -> driverXbox.getLeftY() * -1,
              () -> driverXbox.getLeftX() * -1)
          .withControllerRotationAxis(driverXbox::getRightX)
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    flywheel = new FlywheelSubsystem();
    telemetry.setDrivebase(drivebase);
    climber = new Climb();
    indexer = new IndexerSubsystem();
    hood = new Hood();
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    // Initialize the Climber State
    climber.setState(State.OFF);

    // Set the default auto (do nothing)
    autoChooser.setDefaultOption("Do Nothing", Commands.none());

    // Add a simple auto option to have the robot drive forward for 1 second then stop
    autoChooser.addOption("Drive Forward", drivebase.driveForward().withTimeout(1));

    // Add the options to set which side we are climbing on
    climbSideChooser.addOption("Climb Left", Side.LEFT);
    climbSideChooser.addOption("Climb Right", Side.RIGHT);

    climber.setSide(climbSideChooser.getSelected());

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
    Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    flywheel.setDefaultCommand(new FlywheelCommand(flywheel));
    driverXbox
        .y()
        .onTrue(
            new InstantCommand(() -> flywheel.removeDefaultCommand())
                .andThen(new InstantCommand(() -> flywheel.toggle(), flywheel))); // manual override for the flywheel, toggles between hish and low speed
    manipXbox.x().onTrue(InstanceIntakeSubsystem.EnableIntakeCommand());  // Put intake out on the press of x
    manipXbox.x().whileTrue(InstanceIntakeSubsystem.StartRollersCommand()); // run the rollers while x is pressed
    manipXbox.b().onTrue(InstanceIntakeSubsystem.disableIntakeCommand()); // disable the intake on the press of b
    manipXbox.leftTrigger().whileTrue(new AlignToHub(drivebase, drivebase.getWaypointManager(), "HUB", driverXbox.getLeftX(), driverXbox.getLeftY()).alongWith(new AlignHoodCommand(hood, drivebase, drivebase.getWaypointManager()))); // TODO: Put Align to Hub Command here
    manipXbox.rightTrigger().whileTrue(indexer.spinRollerShooterCommand()); // Enables indexer feeding balls into flywheel
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

  public void updateVision() {
    vision.updatePoseEstimation(drivebase.getSwerveDrive());
  }

  public Vision getVision() {
    return vision;
  }
}
