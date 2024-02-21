package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.BooleanSupplier;

public class IntakeCommand extends Command {
  private final IntakeSubsystem m_intakeSubsystem;
  private final IndexerSubsystem m_indexerSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;
  private final ElevatorSubsystem m_elevatorSubsystem;
  private final BooleanSupplier rt;
  private boolean hasNote;

  public IntakeCommand(
      BooleanSupplier rt,
      IntakeSubsystem intake,
      IndexerSubsystem indexer,
      ShooterSubsystem shooter,
      ElevatorSubsystem elevator) {
    m_intakeSubsystem = intake;
    m_indexerSubsystem = indexer;
    m_shooterSubsystem = shooter;
    m_elevatorSubsystem = elevator;
    this.rt = rt;
    addRequirements(intake, indexer, shooter, elevator);
  }

  @Override
  public void initialize() {
    m_shooterSubsystem.indexAngle();
    m_elevatorSubsystem.defaultPosition();
    hasNote = false;
  }

  @Override
  public void execute() {
    hasNote = false;
    if (hasNote) {
      m_indexerSubsystem.setLowerIndexer(IndexerConstants.kIndexSpeed);
      m_indexerSubsystem.setUpperIndexer(IndexerConstants.kIndexSpeed);
      m_intakeSubsystem.setSpeed(IndexerConstants.kIndexSpeed);
    } else {
      m_indexerSubsystem.setLowerIndexer(0.1);
      m_indexerSubsystem.setUpperIndexer(0);
      m_intakeSubsystem.setSpeed(1);
      if (m_indexerSubsystem.getBeamBreak()) {
        hasNote = true;
      }
    }
  }

  @Override
  public boolean isFinished() {
    return hasNote ? !m_indexerSubsystem.getBeamBreak() : !rt.getAsBoolean();
    // return false;
  }

  @Override
  public void end(boolean interrupted) {
    m_indexerSubsystem.setLowerIndexer(0);
    m_intakeSubsystem.setSpeed(0);
  }
}
