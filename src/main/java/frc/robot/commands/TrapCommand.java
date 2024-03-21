package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LEDMode;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.BooleanSupplier;

public class TrapCommand extends Command {
  private final ShooterSubsystem m_shooterSubsystem;
  private final ElevatorSubsystem m_elevatorSubsystem;
  private final IndexerSubsystem m_indexerSubsystem;
  private final LEDSubsystem m_ledSubsystem;
  private final BooleanSupplier scoring;
  private final BooleanSupplier stop;

  public TrapCommand(
      BooleanSupplier scoring,
      ShooterSubsystem shooter,
      ElevatorSubsystem elevator,
      IndexerSubsystem indexer,
      LEDSubsystem led,
      BooleanSupplier stop) {
    m_shooterSubsystem = shooter;
    m_elevatorSubsystem = elevator;
    m_indexerSubsystem = indexer;
    m_ledSubsystem = led;
    this.scoring = scoring;
    this.stop = stop;
    addRequirements(shooter, elevator, indexer, led);
  }

  @Override
  public void initialize() {
    m_elevatorSubsystem.trapPosition();
    m_elevatorSubsystem.setBrake();
    m_shooterSubsystem.trapAngle();
    m_shooterSubsystem.setPower(0.18);
    m_ledSubsystem.setMode(LEDMode.kRainbow);
  }

  @Override
  public void execute() {
    if (scoring.getAsBoolean()) {
      m_indexerSubsystem.setUpperIndexer(0.2);
    } else {
      m_indexerSubsystem.setUpperIndexer(0);
    }

    if (stop.getAsBoolean()) {
      m_shooterSubsystem.setPower(0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_elevatorSubsystem.defaultPosition();
    m_shooterSubsystem.indexAngle();
    m_shooterSubsystem.setPower(0);
    m_indexerSubsystem.setUpperIndexer(0);
  }
}
