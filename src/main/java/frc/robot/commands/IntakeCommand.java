package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeCommand extends Command {
  private final IntakeSubsystem m_intakeSubsystem;
  private final IndexerSubsystem m_indexerSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;

  public IntakeCommand(IntakeSubsystem intake, IndexerSubsystem indexer, ShooterSubsystem shooter) {
    m_intakeSubsystem = intake;
    m_indexerSubsystem = indexer;
    m_shooterSubsystem = shooter;
    addRequirements(intake, indexer, shooter);
  }

  @Override
  public void initialize() {
    m_indexerSubsystem.setLowerIndexer(1);
    m_indexerSubsystem.setUpperIndexer(1);
    m_intakeSubsystem.setSpeed(1);
    m_shooterSubsystem.indexAngle();
  }

  @Override
  public boolean isFinished() {
    // return m_indexerSubsystem.getBeamBreak();
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    m_indexerSubsystem.setLowerIndexer(0);
    m_indexerSubsystem.setLowerIndexer(0);
    m_intakeSubsystem.setSpeed(0);
  }
}
