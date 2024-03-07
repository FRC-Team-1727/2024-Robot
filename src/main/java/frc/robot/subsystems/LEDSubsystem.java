package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;

public class LEDSubsystem extends SubsystemBase {
  private final AddressableLED led;
  private final AddressableLEDBuffer buffer;
  private final AddressableLEDSim sim;
  private int animStart;
  private LEDMode mode;

  public LEDSubsystem() {
    animStart = 0;
    mode = LEDMode.kDefault;

    led = new AddressableLED(0);
    buffer = new AddressableLEDBuffer(20);

    led.setLength(buffer.getLength());
    led.setData(buffer);
    led.start();

    sim = AddressableLEDSim.createForChannel(0);
    sim.setRunning(true);
  }

  public void setMode(LEDMode mode) {
    if (this.mode != mode) {
      this.mode = mode;
      animStart = 0;
    }
  }

  @Override
  public void periodic() {
    switch (mode) {
      case kDefault:
        defaultPattern();
        break;
      case kRainbow:
        rainbow();
        break;
      case kEmpty:
        setColor(Color.kRed);
        break;
      case kIndexing:
        setColor(Color.kRed);
        blink();
        break;
      case kIndexed:
        setColor(Color.kGreen);
        break;
      case kReady:
        setColor(Color.kBlue);
        break;
      case kShooting:
        setColor(Color.kBlue);
        shootingPattern();
        break;
    }
    led.setData(buffer);
  }

  private void rainbow() {
    for (int i = 0; i < buffer.getLength(); i++) {
      int hue = animStart + (i * 180 / buffer.getLength()) % 180;
      buffer.setHSV(i, hue, 255, 255);
    }
    animStart += 3;
    animStart %= 180;
  }

  private void defaultPattern() {
    for (int i = 0; i < buffer.getLength(); i++) {
      int value = ((int) animStart + (i * 255 / buffer.getLength())) % 510;
      if (value > 255) {
        value = 510 - value;
      }
      Optional<Alliance> color = DriverStation.getAlliance();
      buffer.setHSV(i, !color.isPresent() || color.get() == Alliance.Red ? 0 : 120, 255, value);
    }
    animStart += 7;
    animStart %= 510;
  }

  private void shootingPattern() {
    for (int i = animStart / 4 % 4; i < buffer.getLength(); i += 4) {
      buffer.setLED(i, Color.kBlack);
    }
    animStart++;
  }

  private void setColor(Color c) {
    for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setLED(i, c);
    }
  }

  private void blink(int period) {
    if (animStart % (period * 2) > period) {
      setColor(Color.kBlack);
    }
    animStart++;
  }

  private void blink() {
    blink(5);
  }

  public Command setRandom() {
    return runOnce(
        () -> {
          setMode(LEDMode.values()[(int) (LEDMode.values().length * Math.random())]);
          System.out.println(mode.toString());
        });
  }
}
