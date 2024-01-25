package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AimCommand extends Command {
    private final DriveSubsystem m_driveSubsystem;
    private final ShooterSubsystem m_shooterSubsystem;

    public AimCommand(DriveSubsystem drive, ShooterSubsystem shooter) {
        m_driveSubsystem = drive;
        m_shooterSubsystem = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        m_shooterSubsystem.startShooter();
    }

    @Override
    public void execute() {
        LimelightResults results = LimelightHelpers.getLatestResults("");
        if (results.targetingResults.targets_Fiducials.length > 0) {

        }
    }
}
