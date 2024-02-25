package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import java.util.function.BooleanSupplier;

public class AutoIntakeCommand extends Command {
  private final IntakeSubsystem m_intakeSubsystem;
  private final IndexerSubsystem m_indexerSubsystem;
  private final ElevatorSubsystem m_elevatorSubsystem;
  private final BooleanSupplier rt;
  private boolean hasNote;

  public AutoIntakeCommand(
      BooleanSupplier rt,
      IntakeSubsystem intake,
      IndexerSubsystem indexer,
      ElevatorSubsystem elevator) {
    m_intakeSubsystem = intake;
    m_indexerSubsystem = indexer;
    m_elevatorSubsystem = elevator;
    this.rt = rt;
    addRequirements(intake, indexer, elevator);
  }

  @Override
  public void initialize() {
    m_elevatorSubsystem.defaultPosition();
    hasNote = false;
  }

  @Override
  public void execute() {
    if (hasNote) {
      m_indexerSubsystem.setLowerIndexer(0.25);
      m_indexerSubsystem.setUpperIndexer(0.1);
      m_intakeSubsystem.setSpeed(IndexerConstants.kIndexSpeed);
    } else {
      m_indexerSubsystem.setLowerIndexer(IndexerConstants.kIndexSpeed);
      m_indexerSubsystem.setUpperIndexer(0.2);
      m_intakeSubsystem.setSpeed(IntakeConstants.kIntakeSpeed);
      if (m_indexerSubsystem.getLowerSensor()) {
        hasNote = true;
      }
    }
  }

  @Override
  public boolean isFinished() {
    return hasNote ? !m_indexerSubsystem.getLowerSensor() : !rt.getAsBoolean();
    // return false;
  }

  @Override
  public void end(boolean interrupted) {
    m_indexerSubsystem.setLowerIndexer(0);
    m_intakeSubsystem.setSpeed(0);
    m_indexerSubsystem.setUpperIndexer(0);
  }
}
