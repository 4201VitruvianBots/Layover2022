// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.utils.Conversions;
import frc.robot.utils.ModuleMap;
import java.util.Map;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class USB {
    public static final int leftJoystick = 0;
    public static final int rightJoystick = 1;
    public static final int xBoxController = 2;
    public static final int testController = 4;
  }

  public static final class Pneumatics {
    public static final int pcmOne = 11;
    public static final PneumaticsModuleType pcmType = PneumaticsModuleType.REVPH; // CTREPCM, REVPH

    public static final int intakePistonForward = pcmType == PneumaticsModuleType.CTREPCM ? 0 : 0;
    public static final int intakePistonReverse = pcmType == PneumaticsModuleType.CTREPCM ? 1 : 1;
  }

  public final class Climber {
    public static final int climbMotorA = 50;
    public static final int climbMotorB = 51;

    public static final int climberLowerLimitOverrideID = 8;
    // public static final int climberUpperLimitOverrideID = 8;

    public static final double climberBottomOutValue = 0;
    public static final double climberTopOutValue = 1;

    public static final int encoderUnitsPerRotation = 2048;
    public static final double climberGearRatio = 72.0 * 72.0 / (10.0 * 36.0);
    // Climber motors have 2 stages: 10T to 72T and 36T to 72T. This ratio is input speed / output
    // speed

    public static final double climberHeightUpperLimit = 4.25; // Upper limit of the climber OUTPUT
    public static final double climberEncoderUpperLimit =
        climberHeightUpperLimit * encoderUnitsPerRotation * climberGearRatio;
    // Upper limit of the climber MOTOR

    public static final double climberEncoderLowerLimit = 0.1;

    // At what point we start slowing down in order to not overshoot upper/lower limits
    public static final double climberHeightSlowdown = 0.75;
    public static final double climberEncoderSlowdown =
        climberHeightSlowdown * encoderUnitsPerRotation * climberGearRatio;

    public static final double maxSpeedLimitsPercent = 0.2;
  }

  public static final class CAN {
    public static final int pigeon = 9;

    public static final int frontLeftCanCoder = 10;
    public static final int frontRightCanCoder = 11;
    public static final int backLeftCanCoder = 12;
    public static final int backRightCanCoder = 13;

    public static final int frontLeftDriveMotor = 20;
    public static final int frontLeftTurnMotor = 21;
    public static final int frontRightDriveMotor = 22;
    public static final int frontRightTurnMotor = 23;
    public static final int backLeftDriveMotor = 24;
    public static final int backLeftTurnMotor = 25;
    public static final int backRightDriveMotor = 26;
    public static final int backRightTurnMotor = 27;
  }

  public static final class Indexer {
    public static final int ejectorMotor = 34;
    public static final int indexerMotor = 35;
    public static final int kickerMotor = 36;
    public static final double kickerGearRatio = 5.0;

    public static final int indexerRearSensor = 0; // Rear = closer to shooter
    public static final int indexerFrontSensor = 1; // Front = closer to intake

    public static final double kKickerKs = 0.63662;
    public static final double kKickerKv = 0.017502;
    public static final double kKickerKa = 0.00035281;

    public static final double radiansPerSecondTolerance = 1.0;

    public static final double falconMaxSpeedRadPerSecond = Conversions.RpmToRadPerSec(6380);

    public static final String colorDetectionLimelightHostname = "limelight-indexer";
    public static final int blueCargoDetectionLimelightPipeline = 0;
    public static final int redCargoDetectionLimelightPipeline = 1;
  }

  public final class Intake {
    public static final int intakeMotor = 30;
    public static final int intakeRollerMotor = 31;
    // public static final int intakeSensor = 0;
  }

  public final class Flywheel {
    public static final int flywheelMotorA = 40;
    public static final int flywheelMotorB = 41;
    public static final double hubToleranceDegrees = 3.0;

    public static final int encoderUnitsPerRotation = 2048;

    //    public static final double kFlywheelKs = 0;
    //    public static final double kFlywheelKv = 0.02001;
    //    public static final double kFlywheelKa = 0.002995;
    public static final double kFlywheelKs =
        0; // old flywheel: 0.53456; // 0.63348; // Jamgo: 0.53456;

    // Volts per (radian per second)
    public static final double kFlywheelKv =
        0.018876; // old flywheel: 0.017092; // 0.01;//0.15784; // Jamgo: 0.017092;

    // Volts per (radian per second squared)
    public static final double kFlywheelKa =
        0.0031698; // old flywheel: 0.0083035; // 0.008;//0.034438; // Jamgo: 0.0083035;

    public static final double lqrRPMThreshold = 10.0;
    public static final double rpmTolerance = 60.0;

    public static final double gearRatio = 1.0;
  }

  public final class LED {
    public static final int CANdleID = 0;
  }

  public static final class Turret {
    public static final int turretMotor = 60;

    public static final int turretHomeSensor = 6;

    public static final int encoderUnitsPerRotation = 2048;
    public static final double canCoderAngleOffset = -329.150;
    public static final double minAngle = -165;
    public static final double maxAngle = 165;

    //    public static final double kF = 0.07;
    //    public static final double kP = 0.1;
    //    public static final double kI = 0.00001;
    //    public static final double kD = 0.0;
    public static final double kF = 0.04;
    public static final double kP = 0.15;
    public static final double kI = 0.0008;
    public static final double kD = 0.0;

    public static final double kErrorBand = 50;
    public static final double kI_Zone = 900;
    public static final double kMaxIAccum = 1000;
    public static final double kCruiseVelocity = 20000;
    public static final double kMotionAcceleration = 30000;

    public static final double kS = 0.83016; // 0.81464;
    public static final double kV = 0.012184; // 0.16822;
    public static final double kA = 0.00036802; // 0.011642;

    public static final double degreeTolerance = 3.0;
    public static final double degreesPerSecondTolerance = 10.0;

    public static final double gearRatio = (34.0 / 16.0) * (170.0 / 16.0);
    // (60.0 / 16.0) * (170.0 / 16.0); TODO Figure out Turret Gear Ratio
  }

  public static final class SwerveDrive {
    public static final double kTrackWidth = Units.inchesToMeters(30);
    public static final double kWheelBase = Units.inchesToMeters(30);

    public static final Map<ModulePosition, Translation2d> kModuleTranslations =
        Map.of(
            ModulePosition.FRONT_LEFT, new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            ModulePosition.FRONT_RIGHT, new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            ModulePosition.BACK_LEFT, new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            ModulePosition.BACK_RIGHT, new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final double frontLeftCANCoderOffset = 94.219;
    public static final double frontRightCANCoderOffset = 132.363;
    public static final double backLeftCANCoderOffset = 284.590;
    public static final double backRightCANCoderOffset = 179.648;

    public static final SwerveDriveKinematics kSwerveKinematics =
        new SwerveDriveKinematics(
            ModuleMap.orderedValues(kModuleTranslations, new Translation2d[0]));

    public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(18);
    public static final double kMaxRotationRadiansPerSecond = Math.PI * 2.0;
    public static final double kMaxRotationRadiansPerSecondSquared = Math.PI * 2.0;

    public static final double kP_X = 0.2;
    public static final double kD_X = 0;
    public static final double kP_Y = 0.2;
    public static final double kD_Y = 0;
    public static final double kP_Theta = 6.0;
    public static final double kD_Theta = 0;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxRotationRadiansPerSecond, kMaxRotationRadiansPerSecondSquared);

    public enum ModulePosition {
      FRONT_LEFT,
      FRONT_RIGHT,
      BACK_LEFT,
      BACK_RIGHT
    }

    public static final class Vision {
      public enum CAMERA_TYPE {
        OAK,
        LIMELIGHT,
        PHOTONVISION
      }

      public enum CAMERA_POSITION {
        GOAL,
        INTAKE,
        LIMELIGHT
      }

      public enum INTAKE_TRACKING_TYPE {
        CARGO,
        LAUNCHPAD
      }

      //    public static double GOAL_CAMERA_MOUNTING_ANGLE_DEGREES = 27.0; // Landing
      public static double GOAL_CAMERA_MOUNTING_ANGLE_DEGREES = 32.0; // Takeoff
      public static double GOAL_CAMERA_MOUNTING_HEIGHT_METERS = 1.0;
      public static double LIMELIGHT_MOUNTING_ANGLE_DEGREES = 35.0;
      public static double LIMELIGHT_MOUNTING_HEIGHT_METERS = Units.inchesToMeters(38.0);
      public static double INTAKE_CAMERA_MOUNTING_ANGLE_DEGREES = 34.3;
      public static double INTAKE_CAMERA_MOUNTING_HEIGHT_METERS = 1.0;
      public static double UPPER_HUB_HEIGHT_METERS = Units.inchesToMeters(104.0);
      public static double UPPER_HUB_RADIUS_METERS = Units.feetToMeters(2);
      public static double LOWER_HUB_HEIGHT_METERS = Units.inchesToMeters(41);
      public static double LOWER_HUB_RADIUS_METERS = Units.inchesToMeters(30.0625);
      public static double CARGO_RADIUS = Units.inchesToMeters(4.75);

      public static double TRAJECTORY_MAX_CARGO_DISTANCE = Units.inchesToMeters(30);
      public static double TRAJECTORY_CARGO_POSITION_TOLERANCE = Units.feetToMeters(1.0);

      public static final Pose2d CARGO_TARMAC_ONE = new Pose2d(7.64, 0.37, new Rotation2d());
      public static final Pose2d CARGO_TARMAC_TWO = new Pose2d(4.64, 2.29, new Rotation2d());
      public static final Pose2d CARGO_TERMINAL = new Pose2d(0.456, 0.81, new Rotation2d());

      public static Pose2d HUB_POSE =
          new Pose2d(Units.feetToMeters(27), Units.feetToMeters(13.5), new Rotation2d());

      /** Offset of the intake camera from the robot's center */
      public static Translation2d INTAKE_CAM_TRANSLATION =
          new Translation2d(
              Units.inchesToMeters(14),
              0); // TODO should this be negative, since the intake is in the back?

      /** Ofset of the intake's center from the robot's center */
      public static Translation2d INTAKE_TRANSLATION =
          new Translation2d(
              Units.inchesToMeters(-24),
              0); // TODO should this be negative, since the intake is in the back?

      public static double INTAKE_H_FOV = Units.degreesToRadians(69);
      public static double INTAKE_DETECTION_DISTANCE = Units.inchesToMeters(1.0);

      public static double MIN_SHOOTING_DISTANCE = Units.feetToMeters(5);
      public static double MAX_SHOOTING_DISTANCE = Units.feetToMeters(20);

      public static String VISION_SERVER_IP = "10.42.1.12";
      public static String LIMELIGHT_IP = "10.42.1.11";
    }
  }

  public static final class Vision {
    public enum CAMERA_TYPE {
      OAK,
      LIMELIGHT,
      PHOTONVISION
    }

    public enum CAMERA_POSITION {
      GOAL,
      INTAKE,
      LIMELIGHT
    }

    public enum INTAKE_TRACKING_TYPE {
      CARGO,
      LAUNCHPAD
    }

    //    public static double GOAL_CAMERA_MOUNTING_ANGLE_DEGREES = 27.0; // Landing
    public static double GOAL_CAMERA_MOUNTING_ANGLE_DEGREES = 32.0; // Takeoff
    public static double GOAL_CAMERA_MOUNTING_HEIGHT_METERS = 1.0;
    public static double LIMELIGHT_MOUNTING_ANGLE_DEGREES = 36.0;
    public static double LIMELIGHT_MOUNTING_HEIGHT_METERS = Units.inchesToMeters(38.0);
    public static double INTAKE_CAMERA_MOUNTING_ANGLE_DEGREES = 34.3;
    public static double INTAKE_CAMERA_MOUNTING_HEIGHT_METERS = 1.0;
    public static double UPPER_HUB_HEIGHT_METERS = Units.inchesToMeters(104.0);
    public static double UPPER_HUB_RADIUS_METERS = Units.feetToMeters(2);
    public static double LOWER_HUB_HEIGHT_METERS = Units.inchesToMeters(41);
    public static double LOWER_HUB_RADIUS_METERS = Units.inchesToMeters(30.0625);
    public static double CARGO_RADIUS = Units.inchesToMeters(4.75);

    public static double TRAJECTORY_MAX_CARGO_DISTANCE = Units.inchesToMeters(30);
    public static double TRAJECTORY_CARGO_POSITION_TOLERANCE = Units.feetToMeters(1.0);

    public static final Pose2d CARGO_TARMAC_ONE = new Pose2d(7.64, 0.37, new Rotation2d());
    public static final Pose2d CARGO_TARMAC_TWO = new Pose2d(4.64, 2.29, new Rotation2d());
    public static final Pose2d CARGO_TERMINAL = new Pose2d(0.456, 0.81, new Rotation2d());

    public static Pose2d HUB_POSE =
        new Pose2d(Units.feetToMeters(27), Units.feetToMeters(13.5), new Rotation2d());

    /** Offset of the intake camera from the robot's center */
    public static Translation2d INTAKE_CAM_TRANSLATION =
        new Translation2d(
            Units.inchesToMeters(14),
            0); // TODO should this be negative, since the intake is in the back?

    /** Ofset of the intake's center from the robot's center */
    public static Translation2d INTAKE_TRANSLATION =
        new Translation2d(
            Units.inchesToMeters(-24),
            0); // TODO should this be negative, since the intake is in the back?

    public static double INTAKE_H_FOV = Units.degreesToRadians(69);
    public static double INTAKE_DETECTION_DISTANCE = Units.inchesToMeters(1.0);

    public static double MIN_SHOOTING_DISTANCE = Units.feetToMeters(5);
    public static double MAX_SHOOTING_DISTANCE = Units.feetToMeters(20);

    public static String VISION_SERVER_IP = "10.42.1.12";
    public static String LIMELIGHT_IP = "10.42.1.11";
  }

  public static final class SwerveModule {
    public static final double kDriveMotorGearRatio = 6.12;
    public static final double kTurningMotorGearRatio = 12.8;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(3.94);
    public static final int kFalconEncoderCPR = 2048;
    public static final int kCANCoderCPR = 4096;

    public static final DCMotor kDriveGearbox = DCMotor.getFalcon500(1);
    public static final DCMotor kTurnGearbox = DCMotor.getFalcon500(1);

    public static final double kDriveMotorDistancePerPulse =
        (kWheelDiameterMeters * Math.PI) / (kFalconEncoderCPR * kDriveMotorGearRatio);
    public static final double kTurningMotorDistancePerPulse =
        360.0 / (kFalconEncoderCPR * kTurningMotorGearRatio);
    public static final double kTurningEncoderDistancePerPulse = 360.0 / kCANCoderCPR;

    public static final double ksDriveVoltSecondsPerMeter = 0.667 / 12;
    public static final double kvDriveVoltSecondsSquaredPerMeter = 2.44 / 12;
    public static final double kaDriveVoltSecondsSquaredPerMeter = 0.27 / 12;

    public static final double kvTurnVoltSecondsPerRadian = 1.47; // originally 1.5
    public static final double kaTurnVoltSecondsSquaredPerRadian = 0.348; // originally 0.3
  }

  // 1 = closed-loop control (using sensor feedback) and 0 = open-loop control (no sensor feedback)
  public enum CONTROL_MODE {
    OPENLOOP,
    CLOSEDLOOP
  }
}
