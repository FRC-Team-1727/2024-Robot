package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;

public class VisionSubsystem extends SubsystemBase {
    public VisionSubsystem() {
        setPipeline(0);
    }
    
    public void setPipeline(int id) {
        LimelightHelpers.setPipelineIndex("", id);
    }

    public LimelightResults getResults() {
        return LimelightHelpers.getLatestResults("");
    }

    public Command visionCommand(DriveSubsystem m_driveSubsystem) {
        return run(
            ()->{
                LimelightResults results = LimelightHelpers.getLatestResults("");
                if (results.targetingResults.targets_Fiducials.length > 0) {
                    Pose2d pose = results.targetingResults.getBotPose2d();
                    double latency = results.targetingResults.latency_capture + results.targetingResults.latency_jsonParse + results.targetingResults.latency_pipeline;
                    m_driveSubsystem.addVisionMeasurement(pose, Timer.getFPGATimestamp() - latency);
                    Logger.recordOutput("Vision/Pose", pose);
                    Logger.recordOutput("Vision/Latency", latency);
                }
            }
        );
    }
}
