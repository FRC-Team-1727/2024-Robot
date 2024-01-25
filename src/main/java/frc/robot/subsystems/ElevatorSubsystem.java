package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ElevatorConstants.*;

import java.util.function.IntSupplier;

public class ElevatorSubsystem extends SubsystemBase {
    private double position = 0;
    private final CANSparkMax[] motors = new CANSparkMax[] {
        new CANSparkMax(kElevatorPorts[0], MotorType.kBrushless),
        new CANSparkMax(kElevatorPorts[1], MotorType.kBrushless)
    };

    public ElevatorSubsystem() {
        motors[1].follow(motors[0], true);

        for (CANSparkMax m : motors) {
            SparkPIDController controller = m.getPIDController();
            controller.setP(kP);
            controller.setI(kI);
            controller.setD(kD);
            controller.setFF(kFF);
        }
    }

    private void setPosition(double newPosition) {
        position = newPosition;
        if (position > kMaxPosition) {
            position = kMaxPosition;
        } else if (position < 0) {
            position = 0;
        }
        motors[0].getPIDController().setReference(position, ControlType.kPosition);
    }

    public Command increment(IntSupplier dir) {
        return run(
            ()->{
                setPosition(position + dir.getAsInt() * kElevatorSpeed);
            }
        );
    }
}
