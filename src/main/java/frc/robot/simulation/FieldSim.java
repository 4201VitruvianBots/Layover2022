// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulation;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.simulation.SimConstants.*;
import frc.robot.subsystems.SwerveDrive;

public class FieldSim {
  private final SwerveDrive m_swerveDrive;
  
  private Field2d m_field2d = new Field2d();
  
  public FieldSim(SwerveDrive swerveDrive) {
    m_swerveDrive = swerveDrive;
  }

  public void initSim() {
  }
  
  public Field2d getField2d() {
    return m_field2d;
  }
  
  private void updateRobotPoses() {
    
  }
  
  public void periodic() {
    updateRobotPoses();
    
    if(RobotBase.isSimulation())
      simulationPeriodic();
    
    SmartDashboard.putData("Field2d", m_field2d);
  }
  
  public void simulationPeriodic() {
  }
}
