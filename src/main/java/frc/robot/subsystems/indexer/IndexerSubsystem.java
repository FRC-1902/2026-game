package frc.robot.subsystems.indexer;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {

  private final SparkMax shooterIndexerMotor;
  private final SparkMax rollerIndexerMotor;
  private final RelativeEncoder shooterIndexerEncoder;
  private final RelativeEncoder rollerIndexerEncoder;

  public IndexerSubsystem() {
    shooterIndexerMotor =
        new SparkMax(IndexerConstants.SHOOTER_INDEXER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

    rollerIndexerMotor =
        new SparkMax(IndexerConstants.ROLLER_INDEXER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

    shooterIndexerEncoder = shooterIndexerMotor.getEncoder();
    rollerIndexerEncoder = rollerIndexerMotor.getEncoder();

    SparkMaxConfig rollerConfig = new SparkMaxConfig(); // Configure roller motor (non existent rn)
    rollerConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
    rollerConfig.smartCurrentLimit(30); // change this as needed

    SparkMaxConfig shooterConfig = new SparkMaxConfig(); // Configures shooter indexer motors
    shooterConfig.idleMode(SparkBaseConfig.IdleMode.kCoast);
    shooterConfig.smartCurrentLimit(30); // change this as needed

    rollerIndexerMotor.configure(
        rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    shooterIndexerMotor.configure(
        shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public Command spinRollerShooterCommand() { // Runs the shooter indexer
    return Commands.startEnd(
        () -> {
          shooterIndexerMotor.set(IndexerConstants.SHOOTER_INDEXER_SPEED);
          rollerIndexerMotor.set(IndexerConstants.ROLLER_INDEXER_SPEED);
        },
        () -> {
          shooterIndexerMotor.set(0);
          rollerIndexerMotor.set(0);
        },
        this);
  }

  public void stopRollers() {
    shooterIndexerMotor.set(0);
    rollerIndexerMotor.set(0);
  }

  public Command autoIndexCommand() {
    return Commands.run(
            () -> {
              shooterIndexerMotor.set(IndexerConstants.SHOOTER_INDEXER_SPEED);
              rollerIndexerMotor.set(IndexerConstants.ROLLER_INDEXER_SPEED);
            })
        .withTimeout(2);
  }

  public Command outtakeCommand() { // Runs the shooter indexer motor backwards
    return Commands.startEnd(
        () -> {
          shooterIndexerMotor.set(-IndexerConstants.SHOOTER_INDEXER_SPEED);
          rollerIndexerMotor.set(-IndexerConstants.ROLLER_INDEXER_SPEED);
        },
        () -> {
          shooterIndexerMotor.set(0);
          rollerIndexerMotor.set(0);
        },
        this);
  }

  @Override
  public void periodic() {
    // logs shooter and roller speeds and currents
    SmartDashboard.putNumber("Indexer/Shooter Speed", shooterIndexerMotor.get());
    SmartDashboard.putNumber("Indexer/Roller Speed", rollerIndexerMotor.get());

    SmartDashboard.putNumber("Indexer/Shooter Current", shooterIndexerMotor.getOutputCurrent());
    SmartDashboard.putNumber("Indexer/Roller Current", rollerIndexerMotor.getOutputCurrent());
    // logs shooter and roller velocities in rpm
    SmartDashboard.putNumber("Indexer/Shooter Velocity (RPM)", shooterIndexerEncoder.getVelocity());
    SmartDashboard.putNumber("Indexer/Roller Velocity (RPM)", rollerIndexerEncoder.getVelocity());
  }
}
