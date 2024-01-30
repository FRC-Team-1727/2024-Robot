package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.BooleanSupplier;

public class AmpCommand extends Command {
  private final ShooterSubsystem m_shooterSubsystem;
  private final ElevatorSubsystem m_elevatorSubsystem;
  private final IndexerSubsystem m_indexerSubsystem;
  private final BooleanSupplier scoring;

  public AmpCommand(
      BooleanSupplier scoring,
      ShooterSubsystem shooter,
      ElevatorSubsystem elevator,
      IndexerSubsystem indexer) {
    m_shooterSubsystem = shooter;
    m_elevatorSubsystem = elevator;
    m_indexerSubsystem = indexer;
    this.scoring = scoring;
    addRequirements(shooter, elevator, indexer);
  }

  @Override
  public void initialize() {
    m_elevatorSubsystem.ampPosition();
    m_shooterSubsystem.ampPosition();
  }

  @Override
  public void execute() {
    if (scoring.getAsBoolean()) {
      m_shooterSubsystem.scoreAmp();
      m_indexerSubsystem.setUpperIndexer(0.5);
    } else {
      m_shooterSubsystem.stopShooter();
      m_indexerSubsystem.setUpperIndexer(0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_elevatorSubsystem.defaultPosition();
    m_shooterSubsystem.indexAngle();
  }
}
