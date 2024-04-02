package frc.robot.commands;

import static frc.robot.Constants.IndexerConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDMode;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IndexCommand extends Command {
  private final IndexerSubsystem m_indexerSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;
  private final ElevatorSubsystem m_elevatorSubsystem;
  private final IntakeSubsystem m_intakeSubsystem;
  private final LEDSubsystem m_ledSubsystem;

  public IndexCommand(
      IndexerSubsystem indexer,
      ShooterSubsystem shooter,
      ElevatorSubsystem elevator,
      IntakeSubsystem intake,
      LEDSubsystem led) {
    m_indexerSubsystem = indexer;
    m_shooterSubsystem = shooter;
    m_elevatorSubsystem = elevator;
    m_intakeSubsystem = intake;
    m_ledSubsystem = led;
    addRequirements(indexer, shooter, elevator, intake, led);
  }

  @Override
  public void initialize() {
    m_shooterSubsystem.indexAngle();
    m_elevatorSubsystem.defaultPosition();
    System.out.println("tele indexing");
    m_ledSubsystem.setMode(LEDMode.kIndexing);
    m_intakeSubsystem.setSpeed(0.3);
  }

  @Override
  public void execute() {
    boolean upper = m_indexerSubsystem.getUpperSensor();
    boolean lower = m_indexerSubsystem.getLowerSensor();

    if (!lower && !upper) {
      m_indexerSubsystem.setUpperIndexer(-kIndexSpeed);
      // m_indexerSubsystem.setLowerIndexer(kIndexSpeed);
      m_indexerSubsystem.setRPM(1000);
    } else if (lower) {
      m_indexerSubsystem.setUpperIndexer(0.1);
      // m_indexerSubsystem.setLowerIndexer(kIndexSpeed);
      m_indexerSubsystem.setRPM(1000);
    }
  }

  @Override
  public boolean isFinished() {
    return m_indexerSubsystem.getUpperSensor() && !m_indexerSubsystem.getLowerSensor();
  }

  @Override
  public void end(boolean interrupted) {
    m_indexerSubsystem.setLowerIndexer(0);
    m_indexerSubsystem.setUpperIndexer(0);
    System.out.println("stop tele indexing");
    if (interrupted) {
      m_ledSubsystem.setMode(LEDMode.kEmpty);
      m_intakeSubsystem.setSpeed(0);
    } else {
      m_ledSubsystem.setMode(LEDMode.kIndexed);
      m_intakeSubsystem.setSpeed(0); // -0.3);
    }
  }
}
