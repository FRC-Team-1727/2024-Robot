package frc.robot.commands;

import static frc.robot.Constants.AimingConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LEDMode;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class AimCommand extends Command {
  private final DriveSubsystem m_driveSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;
  private final IndexerSubsystem m_indexerSubsystem;
  private final LEDSubsystem m_ledSubsystem;
  private final BooleanSupplier shooting;
  private PIDController rotationController;
  private double angle;
  private double distance;

  public AimCommand(
      DriveSubsystem drive,
      ShooterSubsystem shooter,
      IndexerSubsystem indexer,
      BooleanSupplier shooting,
      LEDSubsystem led) {
    m_driveSubsystem = drive;
    m_shooterSubsystem = shooter;
    m_indexerSubsystem = indexer;
    m_ledSubsystem = led;
    this.shooting = shooting;
    addRequirements(shooter, indexer, led);
  }

  @Override
  public void initialize() {
    m_shooterSubsystem.startShooter();
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
    Logger.recordOutput("Aiming/AtSpeed", m_shooterSubsystem.atSpeed());
    Logger.recordOutput("Aiming/AtAngle", m_shooterSubsystem.atAngle());

    if (distance < 1.4) {
      // m_shooterSubsystem.subAngle();
    } else {
      m_shooterSubsystem.autoAim(distance);
    }

    // m_shooterSubsystem.goToTestAngle();

    rotationController.setSetpoint(0);
    if (Math.abs(angle) > 2) {
      rotationController.setP(kAimingP);
    } else {
      rotationController.setP(kAimingP * 2);
    }
    m_driveSubsystem.setAimValue(rotationController.calculate(angle));

    if (shooting.getAsBoolean()) {
      m_indexerSubsystem.setUpperIndexer(1);
      m_indexerSubsystem.setLowerIndexer(1);

      m_ledSubsystem.setMode(LEDMode.kShooting);
      // m_shooterSubsystem.logError();
    } else {
      m_indexerSubsystem.setUpperIndexer(0);
      m_indexerSubsystem.setLowerIndexer(0);

      if (m_shooterSubsystem.atSpeed() && m_shooterSubsystem.atAngle()) {
        m_ledSubsystem.setMode(LEDMode.kReady);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    rotationController.close();
    // m_shooterSubsystem.indexAngle();
    // m_shooterSubsystem.stopShooter();
    m_shooterSubsystem.idleShooter();
    m_driveSubsystem.stopAiming();
    m_indexerSubsystem.setUpperIndexer(0);
    m_indexerSubsystem.setLowerIndexer(0);
    m_ledSubsystem.setMode(LEDMode.kEmpty);
  }
}
