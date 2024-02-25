package frc.robot.subsystems;

import static frc.robot.Constants.IndexerConstants.*;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IndexerSubsystem extends SubsystemBase {
  private final CANSparkFlex lowerIndexer =
      new CANSparkFlex(kLowerIndexerPort, MotorType.kBrushless);
  private final CANSparkFlex upperIndexer =
      new CANSparkFlex(kUpperIndexerPort, MotorType.kBrushless);

  private DigitalInput beamBreak = new DigitalInput(kBeamBreakPort);

  public IndexerSubsystem() {
    lowerIndexer.setInverted(true);
    lowerIndexer.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
    upperIndexer.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
    lowerIndexer.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);
    upperIndexer.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);
  }

  public void setLowerIndexer(double spd) {
    lowerIndexer.set(spd);
  }

  public void setUpperIndexer(double spd) {
    upperIndexer.set(spd);
  }

  public boolean getBeamBreak() {
    return !beamBreak.get();
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Indexer/BeamBreak", getBeamBreak());
    // Logger.recordOutput("Indexer/Position", lowerIndexer.getEncoder().getPosition());
  }
}
