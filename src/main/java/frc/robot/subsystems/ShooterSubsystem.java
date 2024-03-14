package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.*;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
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
    controller.setOutputRange(-0.75, 0.75);
    controller.setIZone(0.01);
    controller.setIMaxAccum(0.1, 0);
    flywheel.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
    flywheel.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);
    flywheel.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
  }

  private void setSpeed(int rpm) {
    flywheel.getPIDController().setReference(rpm, ControlType.kVelocity);
    Logger.recordOutput("Shooter/TargetVelocity", rpm);
  }

  public void setPower(double pct) {
    flywheel.set(pct);
  }

  public void startShooter() {
    setSpeed(kShooterSpeed);
    // flywheel.set(0.95);
  }

  public void stopShooter() {
    flywheel.getPIDController().setReference(0, ControlType.kDutyCycle);
    // flywheel.set(0);
  }

  public void idleShooter() {
    // flywheel.getPIDController().setReference(1500, ControlType.kVelocity);
    flywheel.set(0.25);
  }

  public void setAngle(double degrees) {
    if (degrees > kAmpAngle) {
      degrees = kAmpAngle;
    } else if (degrees < kSubAngle) {
      degrees = kSubAngle;
    }
    angler.getPIDController().setReference(degrees, ControlType.kPosition);
    angle = degrees;
    if (Math.abs(angle - angler.getAbsoluteEncoder().getPosition()) > 0.1) {
      angler.getPIDController().setIAccum(0);
    }
  }

  public void indexAngle() {
    setAngle(kIndexAngle);
  }

  public void subAngle() {
    setAngle(kSubAngle);
  }

  public void ampAngle() {
    setAngle(kAmpAngle);
  }

  public void trapAngle() {
    setAngle(kTrapAngle);
  }

  public void podiumAngle() {
    setAngle(kPodiumAngle);
  }

  public void scoreAmp() {
    flywheel.set(kAmpSpeed);
  }

  public boolean atAngle() {
    return Math.abs(angler.getAbsoluteEncoder(Type.kDutyCycle).getPosition() - angle)
        <= kAngleTolerance;
  }

  public boolean atSpeed() {
    return Math.abs(flywheel.getEncoder().getVelocity() - kShooterSpeed) <= kVelocityTolerance;
  }

  public Command increment(DoubleSupplier val) {
    return runOnce(
        () -> {
          setAngle(angle + val.getAsDouble());
        });
  }

  public void aim(double distance) {
    // set angle based on distance from target
    // setAngle(0.0328914 * distance + 0.0900833);
    setAngle(-0.00863736 * distance * distance + 0.0919203 * distance + -0.0124384);
  }

  public void autoAim(double distance) {
    setAngle(-0.0123551 * distance * distance + 0.110271 * distance + -0.0324104);
  }

  public void logError() {
    Logger.recordOutput("Shooter/VelocityError", 6000 - flywheel.getEncoder().getVelocity());
    Logger.recordOutput("Shooter/AngleError", angle - angler.getAbsoluteEncoder().getPosition());
  }

  @Override
  public void periodic() {
    // Logger.recordOutput("Shooter/CurrentVelocity", flywheel.getEncoder().getVelocity());
    Logger.recordOutput("Shooter/TargetAngle", angle);

    // Logger.recordOutput(
    //     "Shooter/CurrentAngle", angler.getAbsoluteEncoder(Type.kDutyCycle).getPosition());
    // Logger.recordOutput("Shooter/AnglerPower", angler.getAppliedOutput());
  }
}
