package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.*;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkFlex flywheel = new CANSparkFlex(kFlywheelPort, MotorType.kBrushless);
  private final CANSparkMax angler = new CANSparkMax(kAnglerPort, MotorType.kBrushless);
  private double angle;

  public ShooterSubsystem() {
    SparkPIDController controller = flywheel.getPIDController();
    controller.setP(kFlywheelP);
    controller.setI(kFlywheelI);
    controller.setD(kFlywheelD);
    controller.setFF(kFlywheelFF);
    controller = angler.getPIDController();
    controller.setP(kAnglerP);
    controller.setI(kAnglerI);
    controller.setD(kAnglerD);
    controller.setFF(kAnglerFF);
    controller.setFeedbackDevice(angler.getAbsoluteEncoder(Type.kDutyCycle));
  }

  private void setSpeed(int rpm) {
    flywheel.getPIDController().setReference(rpm, ControlType.kVelocity);
    Logger.recordOutput("Shooter/TargetVelocity", rpm);
  }

  public void startShooter() {
    // setSpeed(kShooterSpeed);
    flywheel.set(1);
  }

  public void stopShooter() {
    flywheel.getPIDController().setReference(0, ControlType.kDutyCycle);
  }

  public void setAngle(double degrees) {
    angler.getPIDController().setReference(degrees, ControlType.kPosition);
  }

  public void indexAngle() {
    setAngle(kIndexAngle);
  }

  public void subAngle() {
    setAngle(kSubAngle);
  }

  public void ampPosition() {
    setAngle(kAmpAngle);
  }

  public void scoreAmp() {
    flywheel.set(0.5);
  }

  public boolean atAngle() {
    return Math.abs(angler.getAbsoluteEncoder(Type.kDutyCycle).getPosition() - angle)
        <= kAngleTolerance;
  }

  public void aim(double distance) {
    // set angle based on distance from target
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Shooter/CurrentVelocity", flywheel.getEncoder().getVelocity());
    Logger.recordOutput("Shooter/TargetAngle", angle);
    Logger.recordOutput(
        "Shooter/CurrentAngle", angler.getAbsoluteEncoder(Type.kDutyCycle).getPosition());
  }
}
