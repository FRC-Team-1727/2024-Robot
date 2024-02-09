package frc.robot.commands;

import static frc.robot.Constants.AimingConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;
import org.littletonrobotics.junction.Logger;

public class AmpAimCommand extends Command {
  private final DriveSubsystem m_driveSubsystem;
  private PIDController rotationController;
  private PIDController strafeController;

  public AmpAimCommand(DriveSubsystem drive) {
    m_driveSubsystem = drive;
  }

  @Override
  public void initialize() {
    LimelightHelpers.setPipelineIndex("", 2);
    rotationController = new PIDController(kAimingP, kAimingI, kAimingD);
    strafeController = new PIDController(1, 0, 0);
    rotationController.setSetpoint(0);
    strafeController.setSetpoint(0);
  }

  @Override
  public void execute() {
    double angle;
    double strafe;
    if (LimelightHelpers.getTV("")) {
      double[] pose = LimelightHelpers.getTargetPose_RobotSpace("");
      strafe = pose[0];
      angle = pose[4];
    } else {
      angle = 0;
      strafe = 0;
    }

    if (Math.abs(angle) > 2) {
      rotationController.setP(kAimingP);
      strafe = 0;
    } else {
      rotationController.setP(kAimingP * 2);
    }
    m_driveSubsystem.setAimValue(rotationController.calculate(angle));
    double strafeSpeed = strafeController.calculate(-strafe);
    // m_driveSubsystem.setStrafeValue(strafeSpeed);
    if (strafe > 0.1) {
      m_driveSubsystem.setStrafeValue(0.05);
    } else if (strafe < -0.1) {
      m_driveSubsystem.setStrafeValue(-0.05);
    } else {
      m_driveSubsystem.setStrafeValue(0);
    }

    Logger.recordOutput("Aiming/AmpX", strafe);
    Logger.recordOutput("Aiming/AmpYaw", angle);
    Logger.recordOutput("Aiming/StrafeSpeed", strafeSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    rotationController.close();
    m_driveSubsystem.stopAiming();
    m_driveSubsystem.stopStrafing();
    LimelightHelpers.setPipelineIndex("", 0);
  }
}
