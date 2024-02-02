package frc.robot.commands;

import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import org.littletonrobotics.junction.Logger;

public class AimCommand extends Command {
  private final DriveSubsystem m_driveSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;
  private final VisionSubsystem m_visionSubsystem;
  private Translation2d pose;
  private final PIDController rotationController;

  public AimCommand(DriveSubsystem drive, ShooterSubsystem shooter, VisionSubsystem vision) {
    m_driveSubsystem = drive;
    m_shooterSubsystem = shooter;
    m_visionSubsystem = vision;
    rotationController = new PIDController(kAimingP, kAimingI, kAimingD);
    addRequirements(shooter, vision);
  }

  @Override
  public void initialize() {
    m_shooterSubsystem.startShooter();
    m_visionSubsystem.setPipeline(0);
  }

  @Override
  public void execute() {
    double distance;
    double angle;
    LimelightResults results = m_visionSubsystem.getResults();
    if (results.targetingResults.targets_Fiducials.length > 0) {
      pose = results.targetingResults.getBotPose2d().getTranslation();
      Translation2d target =
          DriverStation.getAlliance().get() == Alliance.Blue
              ? new Translation2d(0, 5.5)
              : new Translation2d(16.54, 5.5);
      distance = pose.getDistance(target);
      angle = target.minus(pose).getAngle().getDegrees();
      Logger.recordOutput("Aiming/Pose", pose);
    } else {
      distance = 0;
      angle = 0;
    }

    Logger.recordOutput("Aiming/Angle", angle);
    Logger.recordOutput("Aiming/Distance", distance);

    if (distance > 2) {
      m_shooterSubsystem.aim(distance);
      rotationController.setSetpoint(angle);
      m_driveSubsystem.setAimValue(rotationController.calculate(m_driveSubsystem.getHeading()));
    } else {
      m_shooterSubsystem.subAngle();
      m_driveSubsystem.stopAiming();
    }
  }

  @Override
  public void end(boolean interrupted) {
    rotationController.close();
    m_shooterSubsystem.indexAngle();
    m_driveSubsystem.stopAiming();
    m_visionSubsystem.setPipeline(0);
  }
}
