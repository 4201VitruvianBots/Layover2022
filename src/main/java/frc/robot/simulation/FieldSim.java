// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulation;

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
    m_field2d.setRobotPose(m_swerveDrive.getPose());
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
