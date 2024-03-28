package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.ShooterSubsystem;

public class AimConstantCommand extends Command {
  private final ShooterSubsystem m_shooterSubsystem;
  private double distance;

  public AimConstantCommand(ShooterSubsystem shooter) {
    m_shooterSubsystem = shooter;
    addRequirements(shooter);
  }

  @Override
  public void execute() {
    boolean hasTag = false;
    if (LimelightHelpers.getTV("limelight-main")) {
      for (RawFiducial f :
          LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-main").rawFiducials) {
        if (f.id == 4 || f.id == 7) {
          distance = f.distToRobot;
          hasTag = true;
        }
      }
    }
    if (!hasTag) {
      distance = 0;
    }

    m_shooterSubsystem.autoAim(distance);
    m_shooterSubsystem.startShooter();
  }
}
