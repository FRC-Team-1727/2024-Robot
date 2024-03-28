package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AmpAimCommand extends Command {
  private final DriveSubsystem m_driveSubsystem;
  private final ElevatorSubsystem m_elevatorSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;

  public AmpAimCommand(DriveSubsystem drive, ElevatorSubsystem elevator, ShooterSubsystem shooter) {
    m_driveSubsystem = drive;
    m_elevatorSubsystem = elevator;
    m_shooterSubsystem = shooter;
    addRequirements(elevator, shooter);
  }

  @Override
  public void initialize() {
    m_shooterSubsystem.ampAimAngle();
    m_elevatorSubsystem.defaultPosition();
  }

  @Override
  public void execute() {
    // m_driveSubsystem.angleToAmp();
    if (LimelightHelpers.getTV("amp")) {
      m_driveSubsystem.setStrafeValue(LimelightHelpers.getBotPose_TargetSpace("amp")[0]);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.indexAngle();
    m_driveSubsystem.stopAiming();
    m_driveSubsystem.stopStrafing();
  }
}
