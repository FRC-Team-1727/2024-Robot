package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LEDMode;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.BooleanSupplier;

public class AmpCommand extends Command {
  private final ShooterSubsystem m_shooterSubsystem;
  private final ElevatorSubsystem m_elevatorSubsystem;
  private final IndexerSubsystem m_indexerSubsystem;
  private final DriveSubsystem m_driveSubsystem;
  private final LEDSubsystem m_ledSubsystem;
  private final BooleanSupplier scoring;

  public AmpCommand(
      BooleanSupplier scoring,
      ShooterSubsystem shooter,
      ElevatorSubsystem elevator,
      IndexerSubsystem indexer,
      DriveSubsystem drive,
      LEDSubsystem led) {
    m_shooterSubsystem = shooter;
    m_elevatorSubsystem = elevator;
    m_indexerSubsystem = indexer;
    m_driveSubsystem = drive;
    m_ledSubsystem = led;
    this.scoring = scoring;
    addRequirements(shooter, elevator, indexer, led);
  }

  @Override
  public void initialize() {
    m_elevatorSubsystem.ampPosition();
    m_shooterSubsystem.ampAngle();
    m_shooterSubsystem.setPower(ShooterConstants.kAmpSpeed);
  }

  @Override
  public void execute() {
    if (scoring.getAsBoolean()) {
      m_indexerSubsystem.setUpperIndexer(ShooterConstants.kAmpSpeed);
    } else {
      m_indexerSubsystem.setUpperIndexer(0);
    }

    m_driveSubsystem.angleToAmp();
  }

  @Override
  public void end(boolean interrupted) {
    m_elevatorSubsystem.defaultPosition();
    m_shooterSubsystem.indexAngle();
    m_shooterSubsystem.setPower(0);
    m_indexerSubsystem.setUpperIndexer(0);
    m_ledSubsystem.setMode(LEDMode.kEmpty);
    m_driveSubsystem.stopAiming();
  }
}
