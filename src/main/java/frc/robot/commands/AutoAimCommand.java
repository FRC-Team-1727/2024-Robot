package frc.robot.commands;

import static frc.robot.Constants.AimingConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import org.littletonrobotics.junction.Logger;

public class AutoAimCommand extends Command {
  private final DriveSubsystem m_driveSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;
  private PIDController rotationController;
  private double angle;
  private double distance;

  public AutoAimCommand(DriveSubsystem drive, ShooterSubsystem shooter) {
    m_driveSubsystem = drive;
    m_shooterSubsystem = shooter;
    addRequirements(shooter, drive);
  }

  @Override
  public void initialize() {
    rotationController = new PIDController(kAimingP, kAimingI, kAimingD);
  }

  @Override
  public void execute() {
    boolean hasTag = false;
    if (LimelightHelpers.getTV("")) {
      for (RawFiducial f : LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("").rawFiducials) {
        if (f.id == 4 || f.id == 7) {
          angle = f.txnc;
          distance = f.distToRobot;
          hasTag = true;
        }
      }
    }
    if (!hasTag) {
      distance = 0;
      angle = 0;
    }

    Logger.recordOutput("Aiming/Angle", angle);
    Logger.recordOutput("Aiming/Distance", distance);

    if (distance < 1.4) {
      // m_shooterSubsystem.subAngle();
    } else {
      m_shooterSubsystem.autoAim(distance);
    }

    rotationController.setSetpoint(0);
    if (Math.abs(angle) > 2) {
      rotationController.setP(kAimingP);
    } else {
      rotationController.setP(kAimingP * 2);
    }
    m_driveSubsystem.autoRotate(rotationController.calculate(angle));
  }

  @Override
  public void end(boolean interrupted) {
    rotationController.close();
    m_driveSubsystem.stopAiming();
  }
}
