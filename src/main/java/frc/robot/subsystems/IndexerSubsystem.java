package frc.robot.subsystems;

import static frc.robot.Constants.IndexerConstants.*;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    lowerIndexer.setInverted(false);
    lowerIndexer.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 500);
    upperIndexer.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 500);
    lowerIndexer.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 500);
    upperIndexer.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 500);
    lowerIndexer.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
    upperIndexer.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
    lowerIndexer.setSmartCurrentLimit(40);
    upperIndexer.setSmartCurrentLimit(40);
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

  public void setRPM(int rpm) {
    lowerIndexer.getPIDController().setReference(rpm, ControlType.kVelocity);
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
    // Logger.recordOutput("Indexer/RPM", lowerIndexer.getEncoder().getVelocity());
    SmartDashboard.putBoolean("Lower Sensor", getLowerSensor());
    SmartDashboard.putBoolean("Upper Sensor", getUpperSensor());
  }
}
