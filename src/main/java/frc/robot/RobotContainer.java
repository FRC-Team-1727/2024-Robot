// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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

    // autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("None", Commands.none());
    autoChooser.addOption("Two Note", new PathPlannerAuto("two_note"));
    autoChooser.addOption("Four Note", new PathPlannerAuto("four_note"));
    autoChooser.addOption("Five Note", new PathPlannerAuto("five_note"));
    autoChooser.addOption("Aim Test", new PathPlannerAuto("aim_test"));
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
    // intaking
    m_driverController
        .rightTrigger()
        .whileTrue(
            new IntakeCommand(
                m_intakeSubsystem,
                m_indexerSubsystem,
                m_shooterSubsystem,
                m_elevatorSubsystem,
                m_ledSubsystem));
    // indexing
    new Trigger(m_indexerSubsystem::getLowerSensor)
        .and(() -> !DriverStation.isAutonomousEnabled())
        .onTrue(
            new IndexCommand(
                m_indexerSubsystem, m_shooterSubsystem, m_elevatorSubsystem, m_ledSubsystem));
    m_driverController
        .rightTrigger()
        .onFalse(
            new IndexCommand(
                    m_indexerSubsystem, m_shooterSubsystem, m_elevatorSubsystem, m_ledSubsystem)
                .onlyIf(m_indexerSubsystem::getLowerSensor));
    // speaker aiming
    m_driverController
        .leftBumper()
        .and(m_driverController.rightBumper().negate())
        .whileTrue(
            new AimCommand(
                m_driveSubsystem,
                m_shooterSubsystem,
                m_indexerSubsystem,
                () -> m_driverController.leftTrigger().getAsBoolean(),
                m_ledSubsystem));
    // podium shot
    m_driverController
        .leftBumper()
        .and(m_driverController.rightBumper())
        .whileTrue(
            new PodiumCommand(
                m_shooterSubsystem,
                m_indexerSubsystem,
                m_elevatorSubsystem,
                () -> m_driverController.leftTrigger().getAsBoolean()));
    // amp scoring
    m_driverController
        .rightBumper()
        .and(m_driverController.leftBumper().negate())
        .whileTrue(
            new AmpCommand(
                () -> m_driverController.leftTrigger().getAsBoolean(),
                m_shooterSubsystem,
                m_elevatorSubsystem,
                m_indexerSubsystem));

    m_driverController.b().onTrue(m_driveSubsystem.runOnce(() -> m_driveSubsystem.resetGyro()));

    // m_driverController.y().onTrue(m_climbSubsystem.upPosition());
    // m_driverController.x().onTrue(m_climbSubsystem.downPosition());
    // m_driverController.x().whileTrue(m_climbSubsystem.moveDown());

    // toggle trap position
    m_driverController
        .povCenter()
        .toggleOnFalse(
            new TrapCommand(
                () -> m_driverController.leftTrigger().getAsBoolean(),
                () -> m_driverController.a().getAsBoolean(),
                m_shooterSubsystem,
                m_elevatorSubsystem,
                m_indexerSubsystem));
    // outtake
    m_driverController
        .x()
        .whileTrue(
            Commands.startEnd(
                () -> {
                  m_indexerSubsystem.setUpperIndexer(-1);
                  m_indexerSubsystem.setLowerIndexer(-1);
                  m_intakeSubsystem.setSpeed(-1);
                },
                () -> {
                  m_indexerSubsystem.setUpperIndexer(0);
                  m_indexerSubsystem.setLowerIndexer(0);
                  m_intakeSubsystem.setSpeed(0);
                },
                m_indexerSubsystem,
                m_intakeSubsystem));

    // source intake
    m_driverController
        .rightTrigger()
        .and(m_driverController.leftTrigger())
        .whileTrue(
            new SourceCommand(
                m_shooterSubsystem, m_elevatorSubsystem, m_indexerSubsystem, m_ledSubsystem));

    // zero elevator
    m_driverController
        .back()
        .onTrue(m_elevatorSubsystem.runOnce(m_elevatorSubsystem::resetPosition));

    m_driverController.back().onTrue(m_ledSubsystem.setRandom());

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
    return m_driveSubsystem.runOnce(m_driveSubsystem::resetGyro).andThen(autoChooser.getSelected());
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
                  m_indexerSubsystem.setLowerIndexer(1);
                }),
            Commands.waitSeconds(0.75),
            m_indexerSubsystem.runOnce(
                () -> {
                  m_indexerSubsystem.setUpperIndexer(0);
                  m_indexerSubsystem.setLowerIndexer(0);
                })));
    NamedCommands.registerCommand(
        "intake",
        new AutoIntakeCommand(m_intakeSubsystem, m_indexerSubsystem, m_elevatorSubsystem)
            .raceWith(Commands.waitSeconds(3)));
    NamedCommands.registerCommand(
        "aim",
        new AutoAimCommand(m_driveSubsystem, m_shooterSubsystem)
            .raceWith(Commands.waitSeconds(0.75)));
    NamedCommands.registerCommand(
        "angle_intake", m_shooterSubsystem.runOnce(() -> m_shooterSubsystem.indexAngle()));
  }
}
