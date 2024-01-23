package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ShooterConstants.*;

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkFlex[] motors = new CANSparkFlex[] {
        new CANSparkFlex(kShooterPorts[0], MotorType.kBrushless),
        new CANSparkFlex(kShooterPorts[1], MotorType.kBrushless)
    };

    
    public ShooterSubsystem() {
        motors[1].follow(motors[0], true);

        for (CANSparkFlex m : motors) {
            SparkPIDController controller = m.getPIDController();
            controller.setP(kP);
            controller.setI(kI);
            controller.setD(kD);
            controller.setFF(kFF);
        }
    }

    private void setSpeed(int rpm) {
        motors[0].getPIDController().setReference(rpm, ControlType.kVelocity);
    }
}
