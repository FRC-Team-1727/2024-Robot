package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class DriveConstants {
    public static final HolonomicPathFollowerConfig kConfig =
        new HolonomicPathFollowerConfig(
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
            4.5, // Max module speed, in m/s
            0.4, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options
            // here
            );

    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 15;
    public static final double kMaxAngularSpeed = 3 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2; // 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(22.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(20.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 1;
    public static final int kFrontRightDrivingCanId = 3;
    public static final int kRearLeftDrivingCanId = 5;
    public static final int kRearRightDrivingCanId = 7;

    public static final int kFrontLeftTurningCanId = 2;
    public static final int kFrontRightTurningCanId = 4;
    public static final int kRearLeftTurningCanId = 6;
    public static final int kRearRightTurningCanId = 8;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the
    // bevel pinion
    public static final double kDrivingMotorReduction =
        (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps =
        (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor =
        (kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor =
        ((kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor =
        (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput =
        kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kCoast;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kCoast;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class VortexMotorConstants {
    public static final double kFreeSpeedRpm = 6784;
  }

  public static final class AimingConstants {
    public static final int kCameraAngle = 25;
    public static final double kCameraHeight = 0.1;
    public static final double kTargetHeight = 1.25;
    public static final double kAimingP = 0.01;
    public static final double kAimingI = 0;
    public static final double kAimingD = 0;
    public static final double kStrafingP = 0.01;
  }

  public static final class IntakeConstants {
    public static final int kIntakePort = 16;
    public static final double kIntakeSpeed = 1;
  }

  public static final class ElevatorConstants {
    public static final int[] kElevatorPorts = {9, 10};

    public static final double kP = 0.1;
    public static final double kI = 0;
    public static final double kD = 0.1;
    public static final double kFF = 0;

    public static final int kMaxPosition = 76; // 74;
    public static final double kElevatorSpeed = 5;
    public static final double kAmpPosition = 40;
    public static final double kPodiumPosition = 70; // 60;
    public static final double kTrapPosition = kMaxPosition;
  }

  public static final class ShooterConstants {
    public static final int kFlywheelPort = 12;
    public static final int kAnglerPort = 11;

    public static final double kFlywheelP = 0.002;
    public static final double kFlywheelI = 0;
    public static final double kFlywheelD = 0;
    public static final double kFlywheelFF = 0.00016;

    public static final double kAnglerP = 5; // pre-CMP: 2
    public static final double kAnglerI = 0; // pre-CMP: 0.01
    public static final double kAnglerD = 0;
    public static final double kAnglerFF = 0;

    public static final int kShooterSpeed = 6000;
    public static final double kAmpSpeed = 0.25;
    public static final double kPassSpeed = 0.75;

    public static final double kAngleTolerance = 0.01;
    public static final int kVelocityTolerance = 500;

    public static final double kIndexAngle = 0.154;
    public static final double kSubAngle = 0.13;
    public static final double kAmpAngle = 0.52;
    public static final double kTrapAngle = 0.13;
    public static final double kPodiumAngle = 0.215 + 0.035 - 0.0195; // +.02
    public static final double kAmpAimAngle = 0.479;
    public static final double kPassAngle = kIndexAngle;
  }

  public static final class IndexerConstants {
    public static final int kLowerIndexerPort = 14;
    public static final int kUpperIndexerPort = 15;
    public static final int kLowerSensorPort = 0;
    public static final int kUpperSensorPort = 1;
    public static final double kIndexSpeed = 0.5;

    public static final double kP = 0.00001;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kFF = 0.0001625;
  }

  public static final class ClimbConstants {
    public static final int kClimbPort = 13;
    public static final int kMaxPosition = 290;
    public static final int kMinPosition = 25;
    public static final double kP = 0.1;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kFF = 0;
    public static final double kClimbSpeed = 1;
  }
}
