package frc.robot.subsystems;

import static frc.robot.Constants.IndexerConstants.*;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IndexerSubsystem extends SubsystemBase {
  private final CANSparkFlex lowerIndexer =
      new CANSparkFlex(kLowerIndexerPort, MotorType.kBrushless);
  private final CANSparkFlex upperIndexer =
      new CANSparkFlex(kUpperIndexerPort, MotorType.kBrushless);

  private DigitalInput lowerSensor = new DigitalInput(kLowerSensorPort);
  private DigitalInput upperSensor = new DigitalInput(kUpperSensorPort);

  public IndexerSubsystem() {
    lowerIndexer.setInverted(true);
    lowerIndexer.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
    upperIndexer.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
    lowerIndexer.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);
    upperIndexer.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);
    lowerIndexer.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
    upperIndexer.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
    lowerIndexer.setSmartCurrentLimit(50);
    upperIndexer.setSmartCurrentLimit(50);
    SparkPIDController controller = lowerIndexer.getPIDController();
    controller.setP(kP);
    controller.setI(kI);
    controller.setD(kD);
    controller.setFF(kFF);
    controller.setOutputRange(-1, 1);
    lowerIndexer.burnFlash();
    upperIndexer.burnFlash();
  }

  public void setLowerIndexer(double spd) {
    lowerIndexer.set(spd);
  }

  public void setUpperIndexer(double spd) {
    upperIndexer.set(spd);
  }

  public void indexSpeed() {
    lowerIndexer.getPIDController().setReference(3000, ControlType.kVelocity);
  }

  public boolean getLowerSensor() {
    return !lowerSensor.get();
  }

  public boolean getUpperSensor() {
    return !upperSensor.get();
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Indexer/LowerSensor", getLowerSensor());
    Logger.recordOutput("Indexer/UpperSensor", getUpperSensor());
    // Logger.recordOutput("Indexer/Position", lowerIndexer.getEncoder().getPosition());
    Logger.recordOutput("Indexer/RPM", lowerIndexer.getEncoder().getVelocity());
  }
}
