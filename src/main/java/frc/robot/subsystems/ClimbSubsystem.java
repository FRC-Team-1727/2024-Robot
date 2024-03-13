package frc.robot.subsystems;

import static frc.robot.Constants.ClimbConstants.*;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class ClimbSubsystem extends SubsystemBase {
  private int position = 0;
  private final CANSparkMax motor = new CANSparkMax(kClimbPort, MotorType.kBrushless);

  public ClimbSubsystem() {
    SparkPIDController controller = motor.getPIDController();
    controller.setP(kP);
    controller.setI(kI);
    controller.setD(kD);
    controller.setFF(kFF);
    controller.setOutputRange(-1, 1);
    controller.setFeedbackDevice(motor.getEncoder());
    motor.setIdleMode(IdleMode.kBrake);
    motor.setInverted(true);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
    motor.burnFlash();
  }

  private void setPosition(int newPosition) {
    position = newPosition;
    if (position > kMaxPosition) {
      position = kMaxPosition;
    } else if (position < 0) {
      position = 0;
    }
    motor.getPIDController().setReference(position, ControlType.kPosition);
  }

  public Command upPosition() {
    return runOnce(() -> setPosition(kMaxPosition));
  }

  public Command downPosition() {
    return runOnce(
        () -> {
          setPosition(kMinPosition);
          motor.setIdleMode(IdleMode.kBrake);
        });
  }

  public Command zeroClimb() {
    return runOnce(
        () -> {
          motor.getEncoder().setPosition(0);
          setPosition(0);
        });
  }

  public Command manualControl(Trigger up, Trigger down) {
    return run(
        () -> {
          if (up.getAsBoolean()) {
            motor.set(kClimbSpeed);
          } else if (down.getAsBoolean()) {
            motor.set(-kClimbSpeed);
          } else {
            motor.set(0);
          }
        });
  }

  public Command moveSpeed(DoubleSupplier speed) {
    return startEnd(
        () -> {
          motor.set(speed.getAsDouble());
        },
        () -> motor.set(0));
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Climb/Target", position);
    Logger.recordOutput("Climb/Position", motor.getEncoder().getPosition());
  }
}
