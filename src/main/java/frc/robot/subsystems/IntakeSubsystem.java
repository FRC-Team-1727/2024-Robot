package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private final CANSparkMax intake = new CANSparkMax(kIntakePort, MotorType.kBrushless);

  public IntakeSubsystem() {
    intake.setInverted(false);
    intake.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 500);
    intake.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 500);
    intake.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
    intake.setIdleMode(IdleMode.kCoast);
    // intake.setSmartCurrentLimit(40);
    intake.burnFlash();
  }

  public void setSpeed(double spd) {
    intake.set(spd);
  }
}
