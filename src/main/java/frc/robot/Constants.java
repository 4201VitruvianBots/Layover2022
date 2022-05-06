// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
  private final class USB {
    public static final int leftJoystick = 0;
    public static final int rightJoystick = 1;
    public static final int xBoxContorller = 2;
  }
  
  private final class CAN {
    public static final int pidgeon = 9;
    
    public static final int frontLeftDriveMotor = 20;
    public static final int frontLeftTurningMotor = 21;
    public static final int frontRightDriveMotor = 22;
    public static final int frontRightTurningMotor = 23;
    public static final int backLeftDriveMotor = 24;
    public static final int backLeftTurningMotor = 25;
    public static final int backRightDriveMotor = 26;
    public static final int backRightTurningMotor = 27;
  }
  
  private final class SwerveDrive {
    public static final double kTrackWidth = Units.inchesToMeters(30);
    public static final double kWheelBase = Units.inchesToMeters(30);
    
    public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(18);
    public static final double kMaxChassisRotationRadiansPerSecond = Math.PI * 1.5;
  }
  
  private final class SwerveModule {
    public static final double kDriveMotorGearRatio = 6.12;
    public static final double kTurningMotorGearRatio = 12.8;
    public static final int kFalconEncoderCPR = 2048;
    public static final int kCANCoderCPR = 4096;
    
    public static final double kTurningMotorDistancePerPulse = 360.0 / kFalconEncoderCPR;
    public static final double kTurningEncoderDistancePerPulse = 360.0 / kCANCoderCPR;

    public static final double ksDriveVoltSecondsPerMeter = 0.667 / 12;
    public static final double kvDriveVoltSecondsSquaredPerMeter = 2.44 / 12;
    public static final double kaDriveVoltSecondsSquaredPerMeter = 0.27 / 12;

    public static final double kvTurnVoltSecondsPerRadian = 1.47; // originally 1.5
    public static final double kaTurnVoltSecondsSquaredPerRadian = 0.348; // originally 0.3
  }
}
