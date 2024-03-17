package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LEDMode;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SourceCommand extends Command {
  private final ShooterSubsystem m_shooterSubsystem;
  private final ElevatorSubsystem m_elevatorSubsystem;
  private final IndexerSubsystem m_indexerSubsystem;
  private final LEDSubsystem m_ledSubsystem;

  public SourceCommand(
      ShooterSubsystem shooter,
      ElevatorSubsystem elevator,
      IndexerSubsystem indexer,
      LEDSubsystem led) {
    m_shooterSubsystem = shooter;
    m_elevatorSubsystem = elevator;
    m_indexerSubsystem = indexer;
    m_ledSubsystem = led;
    addRequirements(shooter, elevator, indexer, led);
  }

  @Override
  public void initialize() {
    m_elevatorSubsystem.defaultPosition();
    m_shooterSubsystem.indexAngle();
    m_shooterSubsystem.setPower(-0.3);
    m_indexerSubsystem.setUpperIndexer(-0.3);
    m_ledSubsystem.setMode(LEDMode.kSource);
  }

  @Override
  public boolean isFinished() {
    return m_indexerSubsystem.getLowerSensor();
  }

  @Override
  public void end(boolean interrupted) {
    m_elevatorSubsystem.defaultPosition();
    m_shooterSubsystem.indexAngle();
    m_shooterSubsystem.setPower(0);
    m_indexerSubsystem.setUpperIndexer(0);
  }
}
