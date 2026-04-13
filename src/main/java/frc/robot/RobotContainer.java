// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.PrepForShotCommand;
import frc.robot.commands.autonomous.AutoShoot;
import frc.robot.subsystems.ControllerSubsystem;
import frc.robot.subsystems.ControllerSubsystem.Button;
import frc.robot.subsystems.ControllerSubsystem.ControllerName;
import frc.robot.subsystems.Flywheel.FlywheelSubsystem;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;
import frc.robot.systems.field.AllianceFlipUtil;
import frc.robot.systems.shooting.ShotSolutionProvider;
import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  public final IntakeSubsystem intake;
  public final FlywheelSubsystem flywheel;
  public final ShotSolutionProvider shotProvider;
  private final HoodSubsystem hood;
  public final IndexerSubsystem indexer;
  private final ControllerSubsystem controllers = ControllerSubsystem.getInstance();

  final CommandXboxController driverXbox = controllers.getCommandController(ControllerName.DRIVE);
  final CommandXboxController manipXbox = controllers.getCommandController(ControllerName.MANIP);

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase =
      new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));

  private final Vision vision = new Vision();
  // Establish a Sendable Chooser that will be able to be sent to the SmartDashboard, allowing
  // selection of desired auto
  private final SendableChooser<Command> autoChooser;

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular
   * velocity.
   */
  SwerveInputStream driveAngularVelocity =
      SwerveInputStream.of(
              drivebase.getSwerveDrive(),
              () -> driverXbox.getLeftY() * -1,
              () -> driverXbox.getLeftX() * -1)
          .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    intake = new IntakeSubsystem();
    indexer = new IndexerSubsystem();
    hood = new HoodSubsystem();
    shotProvider = new ShotSolutionProvider(drivebase);
    flywheel = new FlywheelSubsystem(shotProvider.effectiveDistance);

    registerPathPlannerNamedCommands();
    drivebase.configurePathPlanner(AllianceFlipUtil::shouldFlip);

    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    drivebase.zeroGyroWithAlliance();

    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.setDefaultOption("Do Nothing", Commands.none());
    autoChooser.addOption(
        "Shoot Preload",
        new AutoShoot(drivebase, shotProvider, hood, flywheel, indexer, driveAngularVelocity));

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
    controllers
        .getTrigger(ControllerName.MANIP, Button.Y)
        .whileTrue(indexer.outtakeCommand().alongWith(flywheel.spinFlywheelBackwards()));

    controllers.getTrigger(ControllerName.MANIP, Button.X).onTrue(intake.enableIntakeCommand());
    controllers.getTrigger(ControllerName.MANIP, Button.B).onTrue(intake.disableIntakeCommand());
    controllers.getTrigger(ControllerName.MANIP, Button.LB).whileTrue(flywheel.spinUpCommand());
    controllers.getTrigger(ControllerName.MANIP, Button.LB).onFalse(flywheel.spinDownCommand());

    manipXbox
        .leftTrigger()
        .whileTrue(
            new PrepForShotCommand(drivebase, shotProvider, hood, flywheel, driveAngularVelocity));
    manipXbox.leftTrigger().onFalse(flywheel.spinDownCommand());

    controllers.getTrigger(ControllerName.MANIP, Button.RB).whileTrue(intake.startRollersCommand());
    manipXbox.rightTrigger().whileTrue(indexer.spinRollerShooterCommand());

    manipXbox
        .leftTrigger()
        .and(new Trigger(flywheel::isInRange))
        .whileTrue(
            Commands.startEnd(
                () -> controllers.setRumble(ControllerName.MANIP, 1.0),
                () -> controllers.setRumble(ControllerName.MANIP, 0.0)));

    controllers
        .getTrigger(ControllerName.DRIVE, Button.X)
        .onTrue(
            drivebase
                .runOnce(drivebase::zeroGyroWithAlliance)
                .alongWith(
                    Commands.runOnce(() -> controllers.vibrate(ControllerName.DRIVE, 1000, 1.0))));
  }

  /*
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Pass in the selected auto from the SmartDashboard as our desired autnomous commmand
    return autoChooser.getSelected();
  }

  private void registerPathPlannerNamedCommands() {
    NamedCommands.registerCommand("enable rollers", Commands.runOnce(intake::startRollers, intake));
    NamedCommands.registerCommand("disable rollers", Commands.runOnce(intake::stopRollers, intake));
    NamedCommands.registerCommand(
        "shoot",
        new AutoShoot(drivebase, shotProvider, hood, flywheel, indexer, driveAngularVelocity));
    NamedCommands.registerCommand("enable intake", intake.enableIntakeCommand());
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
