package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.BooleanSupplier;

public class TrapCommand extends Command {
  private final ShooterSubsystem m_shooterSubsystem;
  private final ElevatorSubsystem m_elevatorSubsystem;
  private final IndexerSubsystem m_indexerSubsystem;
  private final BooleanSupplier scoring;
  private final BooleanSupplier climbing;

  public TrapCommand(
      BooleanSupplier scoring,
      BooleanSupplier climbing,
      ShooterSubsystem shooter,
      ElevatorSubsystem elevator,
      IndexerSubsystem indexer) {
    m_shooterSubsystem = shooter;
    m_elevatorSubsystem = elevator;
    m_indexerSubsystem = indexer;
    this.scoring = scoring;
    this.climbing = climbing;
    addRequirements(shooter, elevator, indexer);
  }

  @Override
  public void initialize() {
    m_elevatorSubsystem.trapPosition();
    m_shooterSubsystem.trapAngle();
  }

  @Override
  public void execute() {
    // if (climbing.getAsBoolean()) {
    //   m_shooterSubsystem.setPower(-0.1);
    // } else {
    //   m_shooterSubsystem.setPower(0.1);
    // }
    if (scoring.getAsBoolean()) {
      m_indexerSubsystem.setUpperIndexer(ShooterConstants.kAmpSpeed);
      m_shooterSubsystem.setPower(ShooterConstants.kAmpSpeed);
    } else {
      m_indexerSubsystem.setUpperIndexer(0);
      m_shooterSubsystem.setPower(0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_elevatorSubsystem.defaultPosition();
    m_shooterSubsystem.indexAngle();
    m_shooterSubsystem.setPower(0);
    m_indexerSubsystem.setUpperIndexer(0);
  }
}
