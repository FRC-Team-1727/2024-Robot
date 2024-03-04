package frc.robot.commands;

import static frc.robot.Constants.IndexerConstants.*;
import static frc.robot.Constants.IntakeConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IndexCommand extends Command {
  private final IntakeSubsystem m_intakeSubsystem;
  private final IndexerSubsystem m_indexerSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;
  private final ElevatorSubsystem m_elevatorSubsystem;

  public IndexCommand(
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
    if (!m_indexerSubsystem.hasNote()) CommandScheduler.getInstance().cancel(this);
  }

  @Override
  public void execute() {
    boolean upper = m_indexerSubsystem.getUpperSensor();
    boolean lower = m_indexerSubsystem.getLowerSensor();

    if (!lower && !upper) {
      m_indexerSubsystem.setUpperIndexer(-kIndexSpeed);
      m_indexerSubsystem.setLowerIndexer(kIndexSpeed);
      m_intakeSubsystem.setSpeed(kIntakeSpeed);
    } else if (lower) {
      m_indexerSubsystem.setUpperIndexer(kIndexSpeed);
      m_indexerSubsystem.setUpperIndexer(kIndexSpeed);
    }
  }

  @Override
  public boolean isFinished() {
    return m_indexerSubsystem.getUpperSensor() && !m_indexerSubsystem.getLowerSensor();
  }

  @Override
  public void end(boolean interrupted) {
    m_indexerSubsystem.setLowerIndexer(0);
    m_intakeSubsystem.setSpeed(0);
    m_indexerSubsystem.setUpperIndexer(0);
    m_indexerSubsystem.setNote(false);
  }
}
