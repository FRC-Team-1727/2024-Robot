package frc.robot.subsystems;

import static frc.robot.Constants.IndexerConstants.*;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {
  private final CANSparkFlex lowerIndexer =
      new CANSparkFlex(kLowerIndexerPort, MotorType.kBrushless);
  private final CANSparkFlex upperIndexer =
      new CANSparkFlex(kUpperIndexerPort, MotorType.kBrushless);

  private final DigitalInput lowerBeamBreak = new DigitalInput(kLowerBeamBreakPort);
  private final DigitalInput upperBeamBreak = new DigitalInput(kUpperBeamBreakPort);

  public IndexerSubsystem() {}

  public void setLowerIndexer(double spd) {
    lowerIndexer.set(spd);
  }

  public void setUpperIndexer(double spd) {
    upperIndexer.set(spd);
  }

  public boolean getLowerBeamBreak() {
    return lowerBeamBreak.get();
  }

  public boolean getUpperBeamBreak() {
    return upperBeamBreak.get();
  }
}
