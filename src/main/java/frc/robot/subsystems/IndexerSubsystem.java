package frc.robot.subsystems;

import static frc.robot.Constants.IndexerConstants.*;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {
  private final CANSparkFlex lowerIndexer =
      new CANSparkFlex(kLowerIndexerPort, MotorType.kBrushless);
  private final CANSparkFlex upperIndexer =
      new CANSparkFlex(kUpperIndexerPort, MotorType.kBrushless);

  private final DigitalInput beamBreak = new DigitalInput(kBeamBreakPort);

  public IndexerSubsystem() {}

  public void setLowerIndexer(double spd) {
    lowerIndexer.set(spd);
  }

  public void setUpperIndexer(double spd) {
    upperIndexer.set(spd);
  }

  public boolean getBeamBreak() {
    return beamBreak.get();
  }

  @Override
  public void periodic() {
      Logger.recordOutput("Indexer/BeamBreak", beamBreak.get());
  }
}
