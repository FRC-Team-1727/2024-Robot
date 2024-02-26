package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SlowIndexCommand extends Command {
  private final IntakeSubsystem m_intakeSubsystem;
  private final IndexerSubsystem m_indexerSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;
  private final ElevatorSubsystem m_elevatorSubsystem;

  public SlowIndexCommand(
      IntakeSubsystem intake,
      IndexerSubsystem indexer,
      ShooterSubsystem shooter,
      ElevatorSubsystem elevator) {
    m_intakeSubsystem = intake;
    m_indexerSubsystem = indexer;
    m_shooterSubsystem = shooter;
    m_elevatorSubsystem = elevator;
    addRequirements(intake, indexer, shooter, elevator);
  }

  @Override
  public void initialize() {
    m_shooterSubsystem.indexAngle();
    m_elevatorSubsystem.defaultPosition();
  }

  @Override
  public void execute() {
    m_indexerSubsystem.setLowerIndexer(0.1);
    m_indexerSubsystem.setUpperIndexer(0.1);
    m_intakeSubsystem.setSpeed(IndexerConstants.kIndexSpeed);
  }

  @Override
  public boolean isFinished() {
    return !m_indexerSubsystem.getLowerSensor();
    // return false;
  }

  @Override
  public void end(boolean interrupted) {
    m_indexerSubsystem.setLowerIndexer(0);
    m_intakeSubsystem.setSpeed(0);
    m_indexerSubsystem.setUpperIndexer(0);
  }
}