package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoIntakeCommand extends Command {
  private final IntakeSubsystem m_intakeSubsystem;
  private final IndexerSubsystem m_indexerSubsystem;
  private final ElevatorSubsystem m_elevatorSubsystem;

  public AutoIntakeCommand(
      IntakeSubsystem intake, IndexerSubsystem indexer, ElevatorSubsystem elevator) {
    m_intakeSubsystem = intake;
    m_indexerSubsystem = indexer;
    m_elevatorSubsystem = elevator;
    addRequirements(intake, indexer, elevator);
  }

  @Override
  public void initialize() {
    m_elevatorSubsystem.defaultPosition();
  }

  @Override
  public void execute() {
    // m_indexerSubsystem.setLowerIndexer(IndexerConstants.kIndexSpeed);
    m_indexerSubsystem.indexSpeed();
    m_indexerSubsystem.setUpperIndexer(0.2);
    m_intakeSubsystem.setSpeed(IntakeConstants.kIntakeSpeed);
  }

  @Override
  public boolean isFinished() {
    return m_indexerSubsystem.getLowerSensor();
  }

  @Override
  public void end(boolean interrupted) {
    m_indexerSubsystem.setLowerIndexer(0);
    m_intakeSubsystem.setSpeed(0);
    m_indexerSubsystem.setUpperIndexer(0);
  }
}
