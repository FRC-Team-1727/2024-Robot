// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AmpCommand;
import frc.robot.commands.IndexCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem();
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();

  // The driver's controller
  CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Register PathPlanner Named Commands for auto
    registerNamedCommands();

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_driveSubsystem.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () ->
                m_driveSubsystem.drive(
                    -MathUtil.applyDeadband(
                        m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getRightX(), OIConstants.kDriveDeadband),
                    true,
                    true),
            m_driveSubsystem));

      m_visionSubsystem.setDefaultCommand(m_visionSubsystem.visionCommand(m_driveSubsystem));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    m_driverController
        .rightTrigger()
        .whileTrue(new IntakeCommand(m_intakeSubsystem, m_indexerSubsystem));
    m_driverController
        .rightTrigger()
        .onFalse(new IndexCommand(m_indexerSubsystem, m_shooterSubsystem));
    m_driverController
        .leftBumper()
        .whileTrue(
            Commands.startEnd(
                () -> {
                  m_shooterSubsystem.startShooter();
                },
                () -> {
                  m_shooterSubsystem.stopShooter();
                },
                m_shooterSubsystem));
    m_driverController
        .rightBumper()
        .whileTrue(
            new AmpCommand(
                () -> m_driverController.leftTrigger().getAsBoolean(),
                m_shooterSubsystem,
                m_elevatorSubsystem,
                m_indexerSubsystem));
    m_driverController.y().whileTrue(m_elevatorSubsystem.increment(() -> 1));
    m_driverController.a().whileTrue(m_elevatorSubsystem.increment(() -> -1));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Commands.none();
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand("start_shooter", m_shooterSubsystem.runOnce(()->m_shooterSubsystem.startShooter()));
    NamedCommands.registerCommand("angle_sub", m_shooterSubsystem.runOnce(()->m_shooterSubsystem.subAngle()));
    NamedCommands.registerCommand("shoot",
      Commands.sequence(
        m_indexerSubsystem.runOnce(()->{
          m_indexerSubsystem.setUpperIndexer(1);
        }),
        Commands.waitSeconds(1),
        m_indexerSubsystem.runOnce(()->{
          m_indexerSubsystem.setUpperIndexer(0);
        })
      )
    );
    NamedCommands.registerCommand("intake", m_intakeSubsystem.runOnce(()->m_intakeSubsystem.setSpeed(1)));
    NamedCommands.registerCommand("stop_intake", m_intakeSubsystem.runOnce(()->m_intakeSubsystem.setSpeed(0)));
    NamedCommands.registerCommand("index", new IndexCommand(m_indexerSubsystem, m_shooterSubsystem));
  }
}
