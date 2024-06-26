package frc.robot.subsystems;

import static frc.robot.Constants.ElevatorConstants.*;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.IntSupplier;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {
  private double position = 0;
  private boolean trapping = false;
  private final CANSparkMax[] motors =
      new CANSparkMax[] {
        new CANSparkMax(kElevatorPorts[0], MotorType.kBrushless),
        new CANSparkMax(kElevatorPorts[1], MotorType.kBrushless)
      };

  public ElevatorSubsystem() {

    for (CANSparkMax m : motors) {
      SparkPIDController controller = m.getPIDController();
      controller.setP(kP);
      controller.setI(kI);
      controller.setD(kD);
      controller.setFF(kFF);
      controller.setOutputRange(-1, 1);
      // controller.setOutputRange(0, 0);
      controller.setFeedbackDevice(m.getEncoder());
      m.setIdleMode(IdleMode.kCoast);
      // m.setSmartCurrentLimit(40);
      m.burnFlash();
      m.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 500);
      m.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 500);
      m.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
    }
  }

  private void setPosition(double newPosition) {
    position = newPosition;
    if (position > kMaxPosition) {
      position = kMaxPosition;
    } else if (position < 0) {
      position = 0;
    }
    motors[0].getPIDController().setReference(-position, ControlType.kPosition);
    motors[1].getPIDController().setReference(position, ControlType.kPosition);
  }

  public Command increment(IntSupplier dir) {
    return run(
        () -> {
          setPosition(position + dir.getAsInt() * kElevatorSpeed);
        });
  }

  public void defaultPosition() {
    setPosition(0);
  }

  public void ampPosition() {
    setPosition(kAmpPosition);
  }

  public void trapPosition() {
    setPosition(kTrapPosition);
  }

  public void podiumPosition() {
    setPosition(kPodiumPosition);
  }

  public void resetPosition() {
    position = 0;
    motors[0].getEncoder().setPosition(0);
    motors[1].getEncoder().setPosition(0);
  }

  public Command zeroElevator() {
    return startEnd(
        () -> {
          motors[0].set(0.05);
          motors[1].set(-0.05);
        },
        () -> {
          motors[0].set(0);
          motors[1].set(0);
          resetPosition();
        });
  }

  public void setBrake() {
    motors[0].setIdleMode(IdleMode.kBrake);
    motors[1].setIdleMode(IdleMode.kBrake);
  }

  public void setTrapping(boolean value) {
    trapping = value;
  }

  public boolean isTrapping() {
    return trapping;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Elevator/TargetPosition", position);
    // Logger.recordOutput("Elevator/CurrentPosition", -motors[0].getEncoder().getPosition());
    // Logger.recordOutput("Elevator/Position2", motors[1].getEncoder().getPosition());
    // Logger.recordOutput("Elevator/Power", motors[0].getAppliedOutput());
    // Logger.recordOutput("Elevator/Power2", motors[1].getAppliedOutput());

    // motors[0].set(-0.6);
    // motors[1].set(0.6);
  }
}
