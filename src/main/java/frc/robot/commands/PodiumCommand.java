package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.BooleanSupplier;

public class PodiumCommand extends Command {
  private final ShooterSubsystem m_shooterSubsystem;
  private final IndexerSubsystem m_indexerSubsystem;
  private final ElevatorSubsystem m_elevatorSubsystem;
  private final BooleanSupplier shooting;

  public PodiumCommand(
      ShooterSubsystem shooter,
      IndexerSubsystem indexer,
      ElevatorSubsystem elevator,
      BooleanSupplier shooting) {
    m_shooterSubsystem = shooter;
    m_indexerSubsystem = indexer;
    m_elevatorSubsystem = elevator;
    this.shooting = shooting;
    addRequirements(shooter, indexer, elevator);
  }

  @Override
  public void initialize() {
    m_shooterSubsystem.startShooter();
    m_shooterSubsystem.podiumAngle();
    m_elevatorSubsystem.podiumPosition();
  }

  @Override
  public void execute() {
    if (shooting.getAsBoolean()) {
      m_indexerSubsystem.setUpperIndexer(1);
      m_indexerSubsystem.setLowerIndexer(0.4);
    } else {
      m_indexerSubsystem.setUpperIndexer(0);
      m_indexerSubsystem.setLowerIndexer(0.4);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.idleShooter();
    m_shooterSubsystem.indexAngle();
    m_indexerSubsystem.setUpperIndexer(0);
    m_indexerSubsystem.setLowerIndexer(0);
    m_elevatorSubsystem.defaultPosition();
  }
}
