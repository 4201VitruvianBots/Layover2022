// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
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

  public static final class SwerveDrive {
    public static final double kTrackWidth = Units.inchesToMeters(30);
    public static final double kWheelBase = Units.inchesToMeters(30);

    public static final SwerveModuleMap<Translation2d> kModuleTranslations = SwerveModuleMap.of(
      ModulePosition.FRONT_LEFT, new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      ModulePosition.FRONT_RIGHT, new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      ModulePosition.BACK_LEFT, new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
      ModulePosition.BACK_RIGHT, new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
    );

    public static final SwerveDriveKinematics kSwerveKinematics =
        new SwerveDriveKinematics(kModuleTranslations.valuesArray(Translation2d.class));

    public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(18);
    public static final double kMaxRotationRadiansPerSecond = Math.PI * 1.5;
    public static final double kMaxRotationRadiansPerSecondSquared = Math.PI * 1.5;

    public static final double kP_X = 0.2;
    public static final double kD_X = 0;
    public static final double kP_Y = 0.2;
    public static final double kD_Y = 0;
    public static final double kP_Theta = 8;
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
    
    /**
     * A convenience class that maps {@link ModulePositions} to any class, e.g. module translations, module states, etc.<p>
     * Also contains functions to convert to and from arrays so that it's easier to use WPILib swerve functions.
     */
    public static class SwerveModuleMap<V> extends HashMap<ModulePosition, V> {
      public SwerveModuleMap() {}
      
      /**
       * Creates a SwerveModuleMap with the contents of a {@link Map}.
       * 
       * @param map Must have {@link ModulePosition} as the key type
       */
      public SwerveModuleMap(Map<ModulePosition, V> map) {
        for (ModulePosition i : map.keySet()) {
          this.put(i, map.get(i));
        }
      }

      /**
       * Creates a SwerveModuleMap from multiple values, in the order specified in the {@link ModulePosition} enum.<p>
       * For instantiation, it's better to use {@link #of(ModulePosition, V, ModulePosition, V, ModulePosition, V, ModulePosition, V) of}{@code (K,V,K,V,K,V,K,V)}
       * for clarity. However, it is useful for processing the output of a WPILib swerve function which returns an array.
       * 
       * @param values Must have at least as many elements as {@ModulePosition} has entries. Any entries after will be ignored.
       */
      @SafeVarargs
      public static <V> SwerveModuleMap<V> of(V... values) {
        SwerveModuleMap<V> map = new SwerveModuleMap<>();
        for (int i = 0; i < ModulePosition.values().length; i++) {
          map.put(ModulePosition.values()[i], values[i]);
        }
        return map;
      }

      /**
       * Creates a SwerveModuleMap mapping four {@link ModulePosition}s and four values.
       */
      public static <V> SwerveModuleMap<V> of(ModulePosition k1, V v1, ModulePosition k2, V v2, ModulePosition k3, V v3, ModulePosition k4, V v4) {
        return SwerveModuleMap.of(Map.of(k1,v1,k2,v2,k3,v3,k4,v4));
      }

      /**
       * Returns the values from the map as a {@link List} in the same order as in the {@link ModulePosition} enum.<p>
       * Use instead of {@link #values() values} because that returns a {@link Collection}, which is not ordered.
       */
      public List<V> orderedValues() {
        ArrayList<V> list = new ArrayList<>();
        for (ModulePosition i : ModulePosition.values()) {
          list.add(get(i));
        }
        return list;
      }
      
      /**
       * Returns the values from the map as an Array in the same order as in the {@link ModulePosition} enum.<p>
       * Useful when a WPILib swerve function requires an array as input
       * 
       * @param clazz The class to output an array of, e.g. {@code modulePositions.valuesArray(Translation2d.class)}. Required because Java can't make an array of generics.
       */
      public V[] valuesArray(Class<V> clazz) { // TODO is clazz needed? Can I just use V[]?
        return (V[])orderedValues().toArray();
      }
    }
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
}
