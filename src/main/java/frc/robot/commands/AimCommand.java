package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AimCommand extends Command {
    private final DriveSubsystem m_driveSubsystem;
    private final ShooterSubsystem m_shooterSubsystem;
    private Translation2d pose;
    private final PIDController rotationController;

    public AimCommand(DriveSubsystem drive, ShooterSubsystem shooter) {
        m_driveSubsystem = drive;
        m_shooterSubsystem = shooter;
        rotationController = new PIDController(0, 0, 0);
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        m_shooterSubsystem.startShooter();
    }

    @Override
    public void execute() {
        double distance;
        double angle;
        LimelightResults results = LimelightHelpers.getLatestResults("");
        if (results.targetingResults.targets_Fiducials.length > 0) {
            pose = results.targetingResults.getBotPose2d().getTranslation();
            Translation2d target = DriverStation.getAlliance().get() == Alliance.Blue ? new Translation2d(0, 5.5) : new Translation2d(16.54, 5.5);
            distance = pose.getDistance(target);
            angle = target.minus(pose).getAngle().getDegrees();
        } else {
            distance = 0;
            angle = 0;
        }
        
        if (distance > 2) {
            m_shooterSubsystem.aim(distance);
            rotationController.setSetpoint(angle);
        } else {
            m_shooterSubsystem.subAngle();
            
        }

    }
}
