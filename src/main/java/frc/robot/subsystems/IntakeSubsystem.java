package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.IntakeConstants.*;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkFlex intake = new CANSparkFlex(kIntakePort, MotorType.kBrushless);

    public IntakeSubsystem() {}

    public void setSpeed(double spd) {
        intake.set(spd);
    }
}
