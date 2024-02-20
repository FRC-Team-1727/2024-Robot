package frc.robot.subsystems;

import static frc.robot.Constants.ClimbConstants.*;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ClimbSubsystem extends SubsystemBase {
  private double position = 0;
  private final CANSparkMax motor = new CANSparkMax(kClimbPort, MotorType.kBrushless);

  public ClimbSubsystem() {
    SparkPIDController controller = motor.getPIDController();
    controller.setP(kP);
    controller.setI(kI);
    controller.setD(kD);
    controller.setFF(kFF);
    controller.setOutputRange(-0.5, 0.5);
    controller.setFeedbackDevice(motor.getEncoder());
    motor.setIdleMode(IdleMode.kBrake);
    motor.setInverted(true);
    motor.burnFlash();
  }

  private void setPosition(int position) {
    this.position = position;
    motor.getPIDController().setReference(position, ControlType.kPosition);
  }

  public Command upPosition() {
    return runOnce(()->setPosition(kMaxPosition));
  }

  public Command downPosition() {
    return runOnce(()->setPosition(0));
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
      }
    );
  }

  @Override
  public void periodic() {
      Logger.recordOutput("Climb/Target", position);
      Logger.recordOutput("Climber/Position", motor.getEncoder().getPosition());
  }
}
