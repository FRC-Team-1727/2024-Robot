// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

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
  private final ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();
  private final LEDSubsystem m_ledSubsystem = new LEDSubsystem();

  // The driver's controller
  CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);

  SendableChooser<Command> autoChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Register PathPlanner Named Commands for auto
    registerNamedCommands();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

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
                        m_driverController.getRightY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getRightX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    true,
                    false),
            m_driveSubsystem));

    m_climbSubsystem.setDefaultCommand(
        m_climbSubsystem.manualControl(m_driverController.y(), m_driverController.a()));
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
        .onTrue(
            new IntakeCommand(
                () -> m_driverController.rightTrigger().getAsBoolean(),
                m_intakeSubsystem,
                m_indexerSubsystem,
                m_shooterSubsystem,
                m_elevatorSubsystem));
    m_driverController
        .leftBumper()
        .and(m_driverController.rightBumper().negate())
        .whileTrue(
            new AimCommand(
                m_driveSubsystem,
                m_shooterSubsystem,
                m_indexerSubsystem,
                () -> m_driverController.leftTrigger().getAsBoolean()));
    m_driverController
        .leftBumper()
        .and(m_driverController.rightBumper())
        .whileTrue(new AmpAimCommand(m_driveSubsystem));
    m_driverController
        .rightBumper()
        .whileTrue(
            new AmpCommand(
                () -> m_driverController.leftTrigger().getAsBoolean(),
                m_shooterSubsystem,
                m_elevatorSubsystem,
                m_indexerSubsystem));

    m_driverController.b().onTrue(m_driveSubsystem.runOnce(() -> m_driveSubsystem.resetGyro()));
    m_driverController
        .x()
        .onTrue(m_elevatorSubsystem.runOnce(() -> m_elevatorSubsystem.resetPosition()));

    // m_driverController.y().onTrue(m_climbSubsystem.upPosition());
    // m_driverController.x().onTrue(m_climbSubsystem.downPosition());
    // m_driverController.x().whileTrue(m_climbSubsystem.moveDown());
    m_driverController
        .povCenter()
        .toggleOnFalse(
            new TrapCommand(
                () -> m_driverController.leftTrigger().getAsBoolean(),
                () -> m_driverController.a().getAsBoolean(),
                m_shooterSubsystem,
                m_elevatorSubsystem,
                m_indexerSubsystem));
    m_driverController
        .x()
        .whileTrue(
            Commands.startEnd(
                () -> {
                  m_indexerSubsystem.setLowerIndexer(-1);
                  m_intakeSubsystem.setSpeed(-1);
                },
                () -> {
                  m_indexerSubsystem.setLowerIndexer(0);
                  m_intakeSubsystem.setSpeed(0);
                },
                m_indexerSubsystem,
                m_intakeSubsystem));

    m_driverController
        .back()
        .whileTrue(new SourceCommand(m_shooterSubsystem, m_elevatorSubsystem, m_indexerSubsystem));
    m_driverController
        .back()
        .onFalse(
            new SlowIndexCommand(
                m_intakeSubsystem, m_indexerSubsystem, m_shooterSubsystem, m_elevatorSubsystem));

    // manual up/down controls
    // m_driverController.y().onTrue(m_shooterSubsystem.increment(() -> 0.005));
    // m_driverController.a().onTrue(m_shooterSubsystem.increment(() -> -0.005));
    // m_driverController.y().whileTrue(m_elevatorSubsystem.increment(() -> 1));
    // m_driverController.a().whileTrue(m_elevatorSubsystem.increment(() -> -1));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return new PathPlannerAuto("left_auto");
    return autoChooser.getSelected();
    // return Autos.preload();
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand(
        "start_shooter", m_shooterSubsystem.runOnce(() -> m_shooterSubsystem.startShooter()));
    NamedCommands.registerCommand(
        "stop_shooter", m_shooterSubsystem.runOnce(() -> m_shooterSubsystem.stopShooter()));
    NamedCommands.registerCommand(
        "angle_sub", m_shooterSubsystem.runOnce(() -> m_shooterSubsystem.subAngle()));
    NamedCommands.registerCommand(
        "shoot",
        Commands.sequence(
            m_indexerSubsystem.runOnce(
                () -> {
                  m_indexerSubsystem.setUpperIndexer(1);
                }),
            Commands.waitSeconds(0.5),
            m_indexerSubsystem.runOnce(
                () -> {
                  m_indexerSubsystem.setUpperIndexer(0);
                })));
    NamedCommands.registerCommand(
        "intake",
        new AutoIntakeCommand(
            () -> true, m_intakeSubsystem, m_indexerSubsystem, m_elevatorSubsystem));
    // NamedCommands.registerCommand("intake", Commands.none());
    NamedCommands.registerCommand(
        "aim",
        new AutoAimCommand(m_driveSubsystem, m_shooterSubsystem)
            .raceWith(Commands.waitSeconds(0.5)));
    NamedCommands.registerCommand(
        "angle_intake", m_shooterSubsystem.runOnce(() -> m_shooterSubsystem.indexAngle()));
  }
}
