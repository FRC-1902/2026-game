package frc.robot.subsystems.indexer;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class indexer extends SubsystemBase {

  private final SparkMax shooterIndexerMotor;
  private final SparkMax rollerIndexerMotor;

  public indexer() {
    shooterIndexerMotor =
        new SparkMax(indexerconstants.SHOOTER_INDEXER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    rollerIndexerMotor =
        new SparkMax(indexerconstants.ROLLER_INDEXER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
  }

  public void startShooterIndexer() {
    shooterIndexerMotor.set(indexerconstants.SHOOTER_INDEXER_SPEED);
  }

  public void stopShooterIndexer() {
    shooterIndexerMotor.stopMotor();
  }

  public void startRollerIndexer() {
    rollerIndexerMotor.set(indexerconstants.ROLLER_INDEXER_SPEED);
  }

  public void stopRollerIndexer() {
    rollerIndexerMotor.stopMotor();
  }
}
