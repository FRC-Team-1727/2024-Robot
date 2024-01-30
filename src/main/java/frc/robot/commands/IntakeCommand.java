package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
  private final IntakeSubsystem m_intakeSubsystem;
  private final IndexerSubsystem m_indexerSubsystem;

  public IntakeCommand(IntakeSubsystem intake, IndexerSubsystem indexer) {
    m_intakeSubsystem = intake;
    m_indexerSubsystem = indexer;
    addRequirements(intake, indexer);
  }

  @Override
  public void initialize() {
    m_indexerSubsystem.setLowerIndexer(1);
    m_intakeSubsystem.setSpeed(1);
  }

  @Override
  public boolean isFinished() {
    return m_indexerSubsystem.getLowerBeamBreak();
  }

  @Override
  public void end(boolean interrupted) {
    m_indexerSubsystem.setLowerIndexer(0);
    m_intakeSubsystem.setSpeed(0);
  }
}
