package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private final CANSparkMax intake = new CANSparkMax(kIntakePort, MotorType.kBrushless);

  public IntakeSubsystem() {
    intake.setInverted(true);
  }

  public void setSpeed(double spd) {
    intake.set(spd);
  }
}
