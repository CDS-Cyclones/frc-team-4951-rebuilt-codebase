// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.TunableValues.TunableNum;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

public final class Constants {
  private Constants() {}

  /**
   * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when
   * running on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and
   * "replay" (log replay from a file).
   */
  public static final Mode simMode = Mode.SIM;

  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  /** Drive subsystem constants */
  public static final class DriveConstants {
    private DriveConstants() {}

    public static final double maxSpeedMetersPerSec = 4.0;
    public static final double odometryFrequency = 100.0; // Hz
    public static final double trackWidth = Units.inchesToMeters(25.5);
    public static final double wheelBase = Units.inchesToMeters(24.25);
    public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);

    public static final Translation2d[] moduleTranslations =
        new Translation2d[] {
          new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
          new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
          new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
          new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
        };

    // Zeroed rotation values for each module, see setup instructions
    public static final Rotation2d frontLeftZeroRotation = new Rotation2d(-0.9685);
    public static final Rotation2d frontRightZeroRotation = new Rotation2d(-1.0605);
    public static final Rotation2d backLeftZeroRotation = new Rotation2d(-1.133);
    public static final Rotation2d backRightZeroRotation = new Rotation2d(1.536);

    // CANivore bus name
    public static final String canBusName = "canivore";

    // Device CAN IDs
    public static final int pigeonCanId = 60;
    public static final int candleCanId = 40;

    public static final int frontLeftDriveCanId = 7;
    public static final int backLeftDriveCanId = 5;
    public static final int frontRightDriveCanId = 3;
    public static final int backRightDriveCanId = 1;

    public static final int frontLeftTurnCanId = 8;
    public static final int backLeftTurnCanId = 6;
    public static final int frontRightTurnCanId = 4;
    public static final int backRightTurnCanId = 2;

    // Drive motor configuration
    public static final int driveMotorCurrentLimit = 80;
    public static final double wheelRadiusMeters = Units.inchesToMeters(1.5);
    public static final double driveMotorReduction =
        (45.0 * 22.0) / (14.0 * 15.0); // MAXSwerve with 14 pinion teeth and 22 spur teeth
    public static final DCMotor driveGearbox = DCMotor.getNeoVortex(1);

    // Drive encoder configuration
    public static final double driveEncoderPositionFactor =
        2 * Math.PI / driveMotorReduction; // Rotor Rotations -> Wheel Radians
    public static final double driveEncoderVelocityFactor =
        (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM -> Wheel Rad/Sec

    // Drive PID configuration
    public static final TunableNum driveKp = new TunableNum("Drive/drive/p", 0.01);
    public static final TunableNum driveKd = new TunableNum("Drive/drive/d", 0.02);
    // TODO: Run FeedForward Characterization
    public static final double driveKs = 0.2;
    public static final double driveKv = 0.1;
    public static final double driveSimP = 0.05;
    public static final double driveSimD = 0.0;
    public static final double driveSimKs = 0.0;
    public static final double driveSimKv = 0.0789;

    // Turn motor configuration
    public static final boolean turnInverted = false;
    public static final int turnMotorCurrentLimit = 40;
    public static final double turnMotorReduction = 9424.0 / 203.0;
    public static final DCMotor turnGearbox = DCMotor.getNeo550(1);

    // Turn encoder configuration
    public static final boolean turnEncoderInverted = true;
    public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
    public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

    // Turn PID configuration
    // Todo: Tune
    public static final TunableNum turnKp = new TunableNum("Drive/turn/p", 2.0);
    public static final TunableNum turnKd = new TunableNum("Drive/turn/d", 0.0);
    public static final double turnSimP = 10.0;
    public static final double turnSimD = 0.2;
    public static final double turnPIDMinInput = 0; // Radians
    public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

    // PathPlanner configuration
    public static final double robotMassKg = 40.28;
    public static final double robotMOI = 4.154939;
    public static final double wheelCOF =
        1.6; // https://www.chiefdelphi.com/t/wildstang-robotics-program-team-111-and-112-build-blog-2025/477716/36

    public static final RobotConfig ppConfig =
        new RobotConfig(
            robotMassKg,
            robotMOI,
            new ModuleConfig(
                wheelRadiusMeters,
                maxSpeedMetersPerSec,
                wheelCOF,
                driveGearbox.withReduction(driveMotorReduction),
                driveMotorCurrentLimit,
                1),
            moduleTranslations);

    // Maple-SIM configuration
    public static final DriveTrainSimulationConfig mapleSimConfig =
        DriveTrainSimulationConfig.Default()
            .withCustomModuleTranslations(moduleTranslations)
            .withRobotMass(Kilogram.of(robotMassKg))
            .withGyro(COTS.ofPigeon2())
            .withSwerveModule(
                new SwerveModuleSimulationConfig(
                    driveGearbox,
                    turnGearbox,
                    driveMotorReduction,
                    turnMotorReduction,
                    Volts.of(0.1),
                    Volts.of(0.1),
                    Meters.of(wheelRadiusMeters),
                    KilogramSquareMeters.of(0.02),
                    wheelCOF));
  }

  public static final class VisionConstants {
    private VisionConstants() {}

    // AprilTag layout
    public static AprilTagFieldLayout aprilTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    // Camera names, must match names configured on coprocessor
    public static String camera0Name = "limelight";
    public static String camera1Name = "limelight-rear";

    // Robot to camera transforms
    // (Not used by Limelight, configure in web UI instead)
    public static Transform3d robotToCamera0 =
        new Transform3d(0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, 0.0));
    public static Transform3d robotToCamera1 =
        new Transform3d(-0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, Math.PI));

    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.75;

    public static Transform3d botToCamTransformSim =
        new Transform3d(
            new Translation3d(-.3, 0, 0), // X is forward in m, z is up in m
            new Rotation3d(0, 0, 0) // facing forward
            );

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors =
        new double[] {
          1.0, // Camera 0
          1.0 // Camera 1
        };

    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor =
        Double.POSITIVE_INFINITY; // No rotation data available
  }

  public static final class OrbitConstants {
    private OrbitConstants() {}

    // TODO: Tune all orbit behavior values to preference and field conditions.
    // Tag ids for red and blue alliance.
    public static final int[] RED_TAG_IDS = {9, 10};
    public static final int[] BLUE_TAG_IDS = {25, 26};

    // Driver input scales tangential motion around the orbit center.
    public static final double JOYSTICK_DEADBAND = 0.1;
    public static final double MAX_TANGENTIAL_SPEED_METERS_PER_SEC = 3.5;
    public static final double RIGHT_OFFSET_METERS = Units.inchesToMeters(3.0);
    // Target orbit radius.
    public static final double TARGET_RADIUS_METERS = 2.0;
    public static final double MIN_CONTROL_RADIUS_METERS = 0.2;
    public static final double RADIUS_KP = 2.0;
    public static final double MAX_RADIAL_SPEED_METERS_PER_SEC = 1.0;
    // Heading control keeps the robot pointed at the tag midpoint while translating.
    public static final double MAX_ANGULAR_SPEED_RAD_PER_SEC = 8.0;
    public static final double HEADING_ERROR_SLOWDOWN_RADIANS = 0.6;
    public static final double MIN_TRANSLATION_SCALE = 0.45;

    public static final double HEADING_KP = 5.0;
    public static final double HEADING_KD = 0.4;
    public static final double HEADING_MAX_VELOCITY = 8.0;
    public static final double HEADING_MAX_ACCELERATION = 20.0;
  }

  public static final class IntakeConstants {
    private IntakeConstants() {}

    public static final int kCanId = 20;
    public static final int kCurrentLimit = 40;
    public static final double intakeSpeed = 0.60;
    public static final double kickerIntakeSpeed = -0.80;
    public static final double outtakeSpeed = -0.60;
    public static final double kickerOuttakeSpeed = 0.60;
    public static final double kShootingSpeed = 0.8;
  }

  public static final class ClimberConstants {
    private ClimberConstants() {}

    public static final int kCanId = 24;
    public static final int kCurrentStallLimit = 40;
    public static final double kSecondsToClimb = 10;
    // TODO: find the degree for climber to clear rung
    public static final double kClearRung = 0.0;
    // TOOD: confirm these conversionfactors make sense lol
    public static final double kAbsolutePositionConversionFactor = 360.0;
    public static final double kAbsoluteVelocityConversionFactor = 6.0;
    public static final double kRelativePositionConversionFactor = 9.0;
    public static final double kRelativeVelocityConversionFactor = 0.15;
  }

  public static final class ShooterConstants {
    private ShooterConstants() {}

    public static final int kMainShooterCANId = 22;
    public static final int kSecondaryShooterCANId = 23;
    public static final int kCurrentLimit = 80;

    public static final TunableNum kShooterMainKp = new TunableNum("Shooter/kP", 0.00023);
    public static final TunableNum kShooterMainKi = new TunableNum("Shooter/kI", 0.0);
    public static final TunableNum kShooterMainKd = new TunableNum("Shooter/kD", 0.0);
    public static final TunableNum kShooterMainKs = new TunableNum("Shooter/kS", 0.1);
    public static final TunableNum kShooterMainKv = new TunableNum("Shooter/kV", 0.00202);

    public static final TunableNum kShootRPM = new TunableNum("Shooter/kShootRPM", 3350);
    public static final double k4mShootRPM = 3350;
    public static final double k2mShootRPM = 2825;
    public static final double k1mShootRPM = 2500;
    public static final double kVelocityToleranceRPM = 100.0;
    public static final TunableNum kAutoShootRPM = new TunableNum("Shooter/kAutoShootRPM", 2500);
    public static final TunableNum kPassRPM = new TunableNum("Shooter/kPassRPM", 3250);

    public static final double kSimLaunchPeriodSeconds = 0.20;
    public static final double kSimLaunchHeightMeters = Units.inchesToMeters(24.0);
    public static final TunableNum kSimReferenceRPM =
        new TunableNum("Shooter/SimReferenceRPM", 4000.0);
    public static final TunableNum kSimLaunchVelocityMetersPerSecond =
        new TunableNum("Shooter/SimLaunchVelocityMetersPerSecond", 6.8);
    public static final TunableNum kSimLaunchAngleDegrees =
        new TunableNum("Shooter/SimLaunchAngleDegrees", 65.0);
    public static final double kSimLaunchForwardOffsetMeters = Units.inchesToMeters(12.0);
  }

  public static final class KickerConstants {
    private KickerConstants() {}

    public static final int kKickerCANId = 21;
    public static final int kKickerCurrentLimit = 40;
    public static final double kKickerPercentage = 0.8;
    public static final double kKickerIndexPercent = 1.0;

    public static final TunableNum kKickerKp = new TunableNum("Kicker/kP", 0.00023);
    public static final TunableNum kKickerKi = new TunableNum("Kicker/kI", 0.0);
    public static final TunableNum kKickerKd = new TunableNum("Kicker/kD", 0.1);
    public static final TunableNum kKickerKs = new TunableNum("Kicker/Ks", 0.1);
    public static final TunableNum kKickerKv = new TunableNum("Kicker/Kv", 0.00202);
    public static final TunableNum kKickerShootRPM = new TunableNum("Kicker/kickerRPM", 4000);
  }
}
