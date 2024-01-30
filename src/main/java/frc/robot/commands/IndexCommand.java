package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IndexCommand extends Command {
  private final IndexerSubsystem m_indexerSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;

  public IndexCommand(IndexerSubsystem indexer, ShooterSubsystem shooter) {
    m_indexerSubsystem = indexer;
    m_shooterSubsystem = shooter;
    addRequirements(indexer, shooter);
  }

  @Override
  public void initialize() {
    if (m_indexerSubsystem.getLowerBeamBreak()) {
      m_shooterSubsystem.indexAngle();
    } else {
      CommandScheduler.getInstance().cancel(this);
    }
  }

  @Override
  public void execute() {
    if (m_shooterSubsystem.atAngle()) {
      m_indexerSubsystem.setLowerIndexer(0.25);
      m_indexerSubsystem.setUpperIndexer(0.25);
    }
  }

  @Override
  public boolean isFinished() {
    return m_indexerSubsystem.getUpperBeamBreak();
  }

  @Override
  public void end(boolean interrupted) {
    m_indexerSubsystem.setLowerIndexer(0);
    m_indexerSubsystem.setUpperIndexer(0);
  }
}
