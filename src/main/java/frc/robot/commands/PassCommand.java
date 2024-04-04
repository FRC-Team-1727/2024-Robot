package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LEDMode;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class PassCommand extends Command {
  private final ShooterSubsystem m_shooterSubsystem;
  private final IndexerSubsystem m_indexerSubsystem;
  private final LEDSubsystem m_ledSubsystem;

  public PassCommand(ShooterSubsystem shooter, IndexerSubsystem indexer, LEDSubsystem led) {
    m_shooterSubsystem = shooter;
    m_indexerSubsystem = indexer;
    m_ledSubsystem = led;
    addRequirements(shooter, indexer, led);
  }

  @Override
  public void initialize() {
    m_shooterSubsystem.passMode();
  }

  @Override
  public void execute() {
    if (m_shooterSubsystem.readyToPass()) {
      m_indexerSubsystem.setUpperIndexer(0.75);
      m_indexerSubsystem.setLowerIndexer(0.75);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.idleShooter();
    m_indexerSubsystem.setUpperIndexer(0);
    m_indexerSubsystem.setLowerIndexer(0);
    m_ledSubsystem.setMode(LEDMode.kEmpty);
  }
}
