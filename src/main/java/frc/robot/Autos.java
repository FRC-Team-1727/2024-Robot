package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Autos {
  public static Command preload() {
    return Commands.sequence(
        NamedCommands.getCommand("start_shooter"),
        NamedCommands.getCommand("angle_sub"),
        NamedCommands.getCommand("shoot"),
        NamedCommands.getCommand("stop_shooter"));
  }
}
