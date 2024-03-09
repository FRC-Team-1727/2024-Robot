package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoIntakeCommand extends Command {
  private final IntakeSubsystem m_intakeSubsystem;
  private final IndexerSubsystem m_indexerSubsystem;
  private final ElevatorSubsystem m_elevatorSubsystem;
  private boolean hasNote;

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
    hasNote = false;
    System.out.println("starting auto intake");
  }

  @Override
  public void execute() {
    if (!hasNote) {
      m_indexerSubsystem.setRPM(3000);
      m_indexerSubsystem.setUpperIndexer(0.2);
      m_intakeSubsystem.setSpeed(IntakeConstants.kIntakeSpeed);
      if (m_indexerSubsystem.getLowerSensor()) {
        hasNote = true;
      }
    } else {
      boolean upper = m_indexerSubsystem.getUpperSensor();
      boolean lower = m_indexerSubsystem.getLowerSensor();

      if (!lower && !upper) {
        m_indexerSubsystem.setUpperIndexer(-IndexerConstants.kIndexSpeed);
        m_indexerSubsystem.setRPM(1000);
      } else if (lower) {
        m_indexerSubsystem.setUpperIndexer(0.1);
        m_indexerSubsystem.setRPM(1000);
      }
    }
  }

  @Override
  public boolean isFinished() {
    return hasNote == true
        && m_indexerSubsystem.getUpperSensor()
        && !m_indexerSubsystem.getLowerSensor();
  }

  @Override
  public void end(boolean interrupted) {
    m_indexerSubsystem.setLowerIndexer(0);
    m_intakeSubsystem.setSpeed(0);
    m_indexerSubsystem.setUpperIndexer(0);
    System.out.println("finished auto intake");
  }
}
