package frc.robot.subsystems;

import static frc.robot.Constants.IndexerConstants.*;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IndexerSubsystem extends SubsystemBase {
  private final CANSparkFlex lowerIndexer =
      new CANSparkFlex(kLowerIndexerPort, MotorType.kBrushless);
  private final CANSparkFlex upperIndexer =
      new CANSparkFlex(kUpperIndexerPort, MotorType.kBrushless);

  private final DigitalInput beamBreak = new DigitalInput(kBeamBreakPort);

  public IndexerSubsystem() {
    lowerIndexer.setInverted(true);
  }

  public void setLowerIndexer(double spd) {
    lowerIndexer.set(spd);
  }

  public void setUpperIndexer(double spd) {
    upperIndexer.set(spd);
  }

  public boolean getBeamBreak() {
    return beamBreak.get();
  }

  public Command index() {
    return new FunctionalCommand(
            () -> setLowerIndexer(-0.5),
            null,
            interrupted -> setLowerIndexer(0),
            () -> !getBeamBreak(),
            this)
        .andThen(
            new FunctionalCommand(
                () -> setLowerIndexer(0.25),
                null,
                interrupted -> setLowerIndexer(0),
                this::getBeamBreak,
                this))
        .andThen(
            new FunctionalCommand(
                () -> {
                  setLowerIndexer(0.25);
                  setUpperIndexer(0.25);
                  lowerIndexer.getEncoder().setPosition(0);
                },
                null,
                interrupted -> {
                  setLowerIndexer(0);
                  setUpperIndexer(0);
                },
                () -> lowerIndexer.getEncoder().getPosition() > 20,
                this));
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Indexer/BeamBreak", beamBreak.get());
    Logger.recordOutput("Indexer/Position", lowerIndexer.getEncoder().getPosition());
  }
}
