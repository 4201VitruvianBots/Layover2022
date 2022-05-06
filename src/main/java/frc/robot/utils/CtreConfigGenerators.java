package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

public final class CtreConfigGenerators { 
  private static TalonFXConfiguration generateTurnMotorConfig() {
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    motorConfig.slot0.kF = 0.0;
    motorConfig.slot0.kP = 0.6;
    motorConfig.slot0.kI = 0.0;
    motorConfig.slot0.kD = 12.0;

    SupplyCurrentLimitConfiguration supplyCurrentLimit = new SupplyCurrentLimitConfiguration(true, 25, 40, 0.1);
    motorConfig.supplyCurrLimit = supplyCurrentLimit;

    motorConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;

    return motorConfig;
  }

  private static TalonFXConfiguration generateDriveMotorConfig() {
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    motorConfig.slot0.kF = 0.0;
    motorConfig.slot0.kP = 0.1;
    motorConfig.slot0.kI = 0.0;
    motorConfig.slot0.kD = 0.0;

    SupplyCurrentLimitConfiguration supplyCurrentLimit = new SupplyCurrentLimitConfiguration(true, 35, 60, 0.1);
    motorConfig.supplyCurrLimit = supplyCurrentLimit;

    motorConfig.openloopRamp = 0.25;
    motorConfig.closedloopRamp = 0.1;

    motorConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;

    return motorConfig;
  }
    
  private static CANCoderConfiguration generateCanCoderConfig() {
    CANCoderConfiguration sensorConfig = new CANCoderConfiguration();

    sensorConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    sensorConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    sensorConfig.sensorTimeBase = SensorTimeBase.PerSecond;

    return sensorConfig;
  }
}
