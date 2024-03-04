package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
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
    m_indexerSubsystem.setNote(false);
  }

  @Override
  public void execute() {
    m_indexerSubsystem.setLowerIndexer(IndexerConstants.kIndexSpeed);
    m_indexerSubsystem.setUpperIndexer(0.2);
    m_intakeSubsystem.setSpeed(IntakeConstants.kIntakeSpeed);
  }

  @Override
  public boolean isFinished() {
    return m_indexerSubsystem.getLowerSensor() || !rt.getAsBoolean();
  }

  @Override
  public void end(boolean interrupted) {
    m_indexerSubsystem.setLowerIndexer(0);
    m_intakeSubsystem.setSpeed(0);
    m_indexerSubsystem.setUpperIndexer(0);
    if (m_indexerSubsystem.getLowerSensor()) m_indexerSubsystem.setNote(true);
  }
}
