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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.TunableValues.TunableNum;
import lombok.RequiredArgsConstructor;
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

    public static final double maxSpeedMetersPerSec = 5.0;
    public static final TunableNum driveToPoseMaxLinearSpeedMetersPerSec =
        new TunableNum("Drive/DriveToPose/maxLinearSpeedMetersPerSec", 3.0);
    public static final TunableNum driveToPoseMaxAngularSpeedRadPerSec =
        new TunableNum("Drive/DriveToPose/maxAngularSpeedRadPerSec", 6.0);
    public static final double odometryFrequency = 100.0; // Hz
    public static final double trackWidth = Units.inchesToMeters(25.5);
    public static final double wheelBase = Units.inchesToMeters(24.25);
    public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
    public static final Translation2d readyShootBlueTranslation = new Translation2d(1.477, 4.032);
    public static final Translation2d readyShootRedTranslation = new Translation2d(15.0, 4.1);
    public static final Rotation2d readyShootRotation = new Rotation2d(Units.degreesToRadians(0));
    public static final double readyShootRotationTolerance = Units.degreesToRadians(8);
    public static final double readyShootRadiusMeters = 0.5;

    private static final double inFrontOfTag = Units.inchesToMeters(60);
    private static final double inFrontOfTagRightOffset = Units.inchesToMeters(10);
    private static final double rightOfTag = Units.inchesToMeters(60);
    private static final double leftOfTag = -Units.inchesToMeters(60);
    private static final double inFrontOfTagSim = Units.inchesToMeters(78);
    private static final double inFrontOfTagClimb = Units.inchesToMeters(63);
    /** An enum to represent all desired field poses of the robot. */
    @RequiredArgsConstructor
    public static enum FieldPose {
      middleScore(25, 10, inFrontOfTag, inFrontOfTagRightOffset, Units.degreesToRadians(0), false),
      leftScore(25, 10, inFrontOfTag, leftOfTag, Units.degreesToRadians(-120), false),
      rightScore(25, 10, inFrontOfTag, rightOfTag, Units.degreesToRadians(140), false),
      climb(32, 16, inFrontOfTagClimb, 0, Units.degreesToRadians(0), false);

      private final int tagBlueId;
      private final int tagRedId;
      private final double away;
      private final double side;
      private final double rotation; // in radians
      private final boolean orientationOnly;

      /**
       * Return ID of the tag the pose is relative to.
       *
       * @return The ID of the tag.
       */
      public int getTagId() {
        return DriverStation.getAlliance().get() == Alliance.Red ? tagRedId : tagBlueId;
      }

      /**
       * Return the pose of the tag the pose is relative to.
       *
       * @return {@link Pose3d} of the tag.
       */
      public Pose3d getTagPose() {
        return Constants.VisionConstants.aprilTagLayout.getTagPose(getTagId()).get();
      }

      /**
       * Return the desired pose of the robot. If orientationOnly is true, this will return null.
       *
       * @return {@link Pose3d} of the robot.
       */
      public Pose3d getDesiredPose() {
        return getDesiredPose(false, false, false);
      }

      /**
       * Return the desired pose of the robot. If orientationOnly is true, this will return null.
       *
       * @param ignoreForwards Whether to ignore the forwards offset.
       * @param ignoreSideways Whether to ignore the sideways offset.
       * @param ignoreRotation Whether to ignore the rotation.
       * @return {@link Pose3d} of the robot.
       */
      public Pose3d getDesiredPose(
          boolean ignoreForwards, boolean ignoreSideways, boolean ignoreRotation) {
        Pose3d tagPose = getTagPose();
        double tagAngle = tagPose.getRotation().toRotation2d().getRadians();
        double tagX = tagPose.getTranslation().getX();
        double tagY = tagPose.getTranslation().getY();

        // Ensure the angle is between 0 and 2pi
        if (tagAngle < 0) {
          tagAngle = 2 * Math.PI + tagAngle;
        }

        double cos = Math.cos(tagAngle);
        double sin = Math.sin(tagAngle);

        double newX = tagX;
        double newY = tagY;
        Pose3d newPose;

        switch (Constants.currentMode) {
          case SIM:
            if (this == climb) {
              newX += away * cos;
              newY += away * sin;
            } else {
              newX += inFrontOfTagSim * cos;
              newY += inFrontOfTagSim * sin;
            }
            break;
          default:
            newX += away * cos;
            newY += away * sin;
        }

        // now do transformation to the left or right of the tag
        newX += side * -sin;
        newY += side * cos;

        newPose =
            new Pose3d(
                new Translation3d(ignoreForwards ? tagX : newX, ignoreForwards ? tagY : newY, 0),
                new Rotation3d(0, 0, ignoreRotation ? tagAngle : tagAngle + rotation));

        return newPose;
      }

      /**
       * Return the desired rotation of the robot.
       *
       * @return {@link Rotation2d} of the robot.
       */
      public Rotation2d getDesiredRotation2d() {
        return getDesiredPose().getRotation().toRotation2d();
      }

      /**
       * Return whether the desired pose is orientation only meaning the robot should only rotate to
       * the desired angle and not move.
       *
       * @return True if the desired pose is orientation only.
       */
      public boolean isOrientationOnly() {
        return orientationOnly;
      }

      @Override
      public String toString() {
        return name();
      }
    }

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

    // TODO: THESE VALUES ARE SIM ONLY, CHANGE FOR REAL ROBOT
    public static final PIDController angleController = new PIDController(1.0, 0.0, 0.0);

    public static final PIDController translationXController = new PIDController(1.2, 0.0, 0.0);

    public static final PIDController translationYController = new PIDController(1.2, 0.0, 0.0);

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
    public static Transform3d botToCamTransformSimRear =
        new Transform3d(
            new Translation3d(-.3, 0, 0), // X is forward in m, z is up in m
            new Rotation3d(0, 0, 180) // facing backwards
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

  public static final class IntakeArmConstants {
    private IntakeArmConstants() {}

    public static final int kCanId = 27;
    public static final int kCurrentLimit = 40;
    public static final boolean kMotorInverted = false; // TODO: Confirm intake arm inversion.
    public static final double kGearReduction = 1.0;
    public static final double kAbsolutePositionConversionFactor = 360.0;
    public static final double kAbsoluteVelocityConversionFactor = 6.0;
    public static final double kPositionConversionFactor = 360.0 / kGearReduction;
    public static final double kVelocityConversionFactor = 6.0 / kGearReduction;
    public static final double kStowedPositionDegrees = 0.0; // TODO: check this
    public static final double kDeployedPositionDegrees = 90.0; // TODO: check this
    public static final double kMaxVelocityDegreesPerSecond = 180.0;
    public static final double kMaxAccelerationDegreesPerSecondSq = 360.0;
    public static final TunableNum kKp =
        new TunableNum("IntakeArm/kP", 0.04); // TODO: Tune the intake arm position P gain.
    public static final TunableNum kKi = new TunableNum("IntakeArm/kI", 0.0);
    public static final TunableNum kKd =
        new TunableNum("IntakeArm/kD", 0.0); // TODO: Tune the intake arm position D gain.
    public static final double kToleranceDegrees = 4.0;
  }

  public static final class IntakeArmKickerConstants {
    private IntakeArmKickerConstants() {}

    public static final int kCanId = 26;
    public static final int kCurrentLimit = 20;
    public static final boolean kMotorInverted = false;
    public static final double kRunPercent = 0.85;
  }

  public static final class ClimberConstants {
    private ClimberConstants() {}

    public static final int kCanId = 24;
    public static final int kCurrentStallLimit = 40;
    public static final double kSecondsToClimb = 3.5;
    public static final double kClearRung = 333.6;
    public static final double kClimbStowedPositionDegrees = 45.0; // TODO: Check this
    public static final double kAbsolutePositionConversionFactor = 360.0;
    public static final double kAbsoluteVelocityConversionFactor = 6.0;
    public static final double kRelativePositionConversionFactor = 9.0;
    public static final double kRelativeVelocityConversionFactor = 0.15;
  }

  public static final class HopperConstants {
    private HopperConstants() {}

    public static final int kCanID = 25;
    public static final int kCurrentLimit = 8;
  }

  public static final class ShooterConstants {
    private ShooterConstants() {}

    public static final int kMainShooterCANId = 22;
    public static final int kSecondaryShooterCANId = 23;
    public static final int kCurrentLimit = 75;

    public static final TunableNum kShooterMainKp = new TunableNum("Shooter/kP", 0.00023);
    public static final TunableNum kShooterMainKi = new TunableNum("Shooter/kI", 0.0);
    public static final TunableNum kShooterMainKd = new TunableNum("Shooter/kD", 0.0);
    public static final TunableNum kShooterMainKs = new TunableNum("Shooter/kS", 0.1);
    public static final TunableNum kShooterMainKv = new TunableNum("Shooter/kV", 0.00202);

    public static final TunableNum kShootRPM = new TunableNum("Shooter/kShootRPM", 2900);
    public static final double k4mShootRPM = 3000;
    public static final double k2mShootRPM = 2825;
    public static final double k1mShootRPM = 2500;
    public static final double kVelocityToleranceRPM = 100.0;
    public static final TunableNum kAutoShootRPM = new TunableNum("Shooter/kAutoShootRPM", 2900);
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
    public static final int kKickerCurrentLimit = 30;
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
