package frc.robot.subsystems.indexer;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {

  private final SparkMax shooterIndexerMotor;
  private final SparkMax rollerIndexerMotor;

  public IndexerSubsystem() {
    shooterIndexerMotor =
        new SparkMax(IndexerConstants.SHOOTER_INDEXER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

    rollerIndexerMotor =
        new SparkMax(IndexerConstants.ROLLER_INDEXER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

    // Configure roller motor for power saving
    SparkMaxConfig rollerConfig = new SparkMaxConfig();
    rollerConfig.idleMode(IdleMode.kCoast);
    rollerConfig.smartCurrentLimit(0); // tune this as needed

    rollerIndexerMotor.configure(
        rollerConfig,
        SparkMax.ResetMode.kNoResetSafeParameters,
        SparkMax.PersistMode.kPersistParameters);
  }

  public void startShooterIndexer() {
    shooterIndexerMotor.set(IndexerConstants.SHOOTER_INDEXER_SPEED);
  }

  public void stopShooterIndexer() {
    shooterIndexerMotor.stopMotor();
  }

  public void startRollerIndexer() {
    rollerIndexerMotor.set(IndexerConstants.ROLLER_INDEXER_SPEED);
  }

  public void stopRollerIndexer() {
    rollerIndexerMotor.stopMotor();
  }

  public void periodic() {
    SmartDashboard.putNumber("Indexer/Shooter Speed Cmd", shooterIndexerMotor.get());
    SmartDashboard.putNumber("Indexer/Roller Speed Cmd", rollerIndexerMotor.get());

    SmartDashboard.putNumber("Indexer/Shooter Current", shooterIndexerMotor.getOutputCurrent());
    SmartDashboard.putNumber("Indexer/Roller Current", rollerIndexerMotor.getOutputCurrent());

    SmartDashboard.putNumber("Indexer/Shooter Voltage", shooterIndexerMotor.getBusVoltage());
    SmartDashboard.putNumber("Indexer/Roller Voltage", rollerIndexerMotor.getBusVoltage());
  }
}
