// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final DataLog m_logger = DataLogManager.getLog();
  
  // The robot's subsystems and commands are defined here...
  private final SwerveDrive m_swerveDrive = new SwerveDrive();
  
  private final SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();
  
  private final FieldSim m_fieldSim = new FieldSim(m_swerveDrive);
  
  public RobotContainer() {
    initializeSubsystems();
    initializeAutoChooser();
    
    // Configure the button bindings
    configureButtonBindings();
  }
  
  public void initializeSubsystems() {

  }
  
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    for (int i = 0; i < leftButtons.length; i++)
      leftButtons[i] = new JoystickButton(leftJoystick, (i + 1));
    for (int i = 0; i < rightButtons.length; i++)
      rightButtons[i] = new JoystickButton(rightJoystick, (i + 1));
    for (int i = 0; i < xBoxButtons.length; i++)
      xBoxButtons[i] = new JoystickButton(xBoxController, (i + 1));
    for (int i = 0; i < xBoxPOVButtons.length; i++)
      xBoxPOVButtons[i] = new POVButton(xBoxController, (i * 90));
    
  }
  
  private void initializeAutoChooser() {
    m_autoChooser.setDefaultOption(
        "Do Nothing",
        new WaitCommand(0));
    
    SmartDashboard.putData("Auto Selector", m_autoChooser);
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoChooser.getSelected();
  }

  public void disabledInit() {
    
  }

  public void disabledPeriodic() {
    
  }

  public void teleopInit() {
    
  }

  public void teleopPeriodic() {
    
  }

  public void autonomousInit() {
    
  }

  public void autonomousPeriodic() {
  }
  
  public void updateFieldSim() {
    m_fieldSim.periodic();
  }
  
  public void simulationInit() {
    m_fieldSim.initSim();
  }

  public void simulationPeriodic() {
    m_fieldSim.simulationPeriodic();
  }  
}
