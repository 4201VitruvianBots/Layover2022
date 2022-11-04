// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.Vision.CAMERA_POSITION;
import frc.robot.commands.SetSwerveDrive;
import frc.robot.commands.auto.DoNothing;
import frc.robot.commands.auto.DriveForward;
import frc.robot.commands.auto.FiveBallAuto;
import frc.robot.commands.auto.TestAuto;
import frc.robot.commands.auto.ThreeBallAutoStart;
import frc.robot.commands.climber.SetClimbState;
import frc.robot.commands.climber.SetClimberOutput;
import frc.robot.commands.flywheel.SetRpmSetpoint;
import frc.robot.commands.flywheel.ShotSelecter;
import frc.robot.commands.indexer.RunIndexer;
import frc.robot.commands.indexer.RunOnlyIndexer;
import frc.robot.commands.intake.ReverseIntakeIndexer;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.swerve.ResetOdometry;
import frc.robot.commands.turret.SetTurretSetpointFieldAbsolute;
import frc.robot.commands.turret.ToggleTurretControlMode;
import frc.robot.commands.turret.ToggleTurretLock;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Controls;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final DataLog m_logger = DataLogManager.getLog();

  // The robot's subsystems and commands are defined here...
  private final Controls m_controls = new Controls();
  private final SwerveDrive m_swerveDrive = new SwerveDrive();
  private final Turret m_turret = new Turret(m_swerveDrive);
  private final Vision m_vision = new Vision(m_controls, m_swerveDrive, m_turret, m_logger);
  private final Flywheel m_flywheel = new Flywheel(m_vision, m_turret);
  private final Intake m_intake = new Intake();
  private final Indexer m_indexer = new Indexer();
  private final SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();
  private final Climber m_climber = new Climber();

  private final FieldSim m_fieldSim = new FieldSim(m_swerveDrive);

  static Joystick leftJoystick = new Joystick(Constants.USB.leftJoystick);
  static Joystick rightJoystick = new Joystick(Constants.USB.rightJoystick);
  static XboxController xBoxController = new XboxController(Constants.USB.xBoxController);
  static PS4Controller testController = new PS4Controller(Constants.USB.testController);

  public Button[] leftButtons = new Button[2];
  public Button[] rightButtons = new Button[2];
  public Button[] xBoxButtons = new Button[10];
  public Button[] xBoxPOVButtons = new Button[4];
  public Button xBoxLeftTrigger, xBoxRightTrigger;

  public RobotContainer() {
    initializeSubsystems();
    initializeAutoChooser();

    // Configure the button bindings
    configureButtonBindings();
  }

  public void initializeSubsystems() {
    // m_swerveDrive.setDefaultCommand(
    //     new SetSwerveDrive(
    //         m_swerveDrive,
    //         () -> -testController.getLeftY(),
    //         () -> -testController.getLeftX(),
    //         () -> -testController.getRightX()));

    m_swerveDrive.setDefaultCommand(
        new SetSwerveDrive(
            m_swerveDrive,
            () -> leftJoystick.getRawAxis(1),
            () -> leftJoystick.getRawAxis(0),
            () -> rightJoystick.getRawAxis(0)));
    m_fieldSim.initSim();

    //     m_intake.setDefaultCommand( TODO: test
    // new RunIntake(m_intake));

    m_climber.setDefaultCommand(
        new SetClimberOutput(m_climber, () -> xBoxController.getRawAxis(5)));

    // m_climber.setDefaultCommand(
    //   new SetClimberOutput(m_climber, () -> xBoxController.getRawAxis(5)));
    m_turret.setDefaultCommand(
        new SetTurretSetpointFieldAbsolute(
            m_turret, m_swerveDrive, m_vision, m_flywheel, m_climber, xBoxController));
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

    xBoxLeftTrigger =
        new Button(
            () -> xBoxController.getLeftTriggerAxis() > 0.2); // getTrigger());// getRawAxis(2));
    xBoxRightTrigger = new Button(() -> xBoxController.getRightTriggerAxis() > 0.2);

    xBoxButtons[0].whileHeld(new SetRpmSetpoint(m_flywheel, m_vision, () -> m_flywheel.tarmacShot));
    xBoxButtons[1].whileHeld(
        new SetRpmSetpoint(m_flywheel, m_vision, () -> m_flywheel.launchpadShot));
    xBoxButtons[3].whileHeld(
        new SetRpmSetpoint(
            m_flywheel,
            m_vision,
            () ->
                ShotSelecter.bestShot(
                    m_vision.getGoalTargetHorizontalDistance(CAMERA_POSITION.LIMELIGHT))));

    xBoxButtons[6].whenPressed(new ToggleTurretControlMode(m_turret));

    xBoxButtons[7].whenPressed(new ToggleTurretLock(m_turret));

    xBoxPOVButtons[2].whileHeld(new ReverseIntakeIndexer(m_intake, m_indexer));
    xBoxPOVButtons[0].whileHeld(new RunIndexer(m_intake, m_indexer, m_flywheel, false));
    xBoxLeftTrigger.whileHeld(new RunIntake(m_intake, m_indexer));
    xBoxLeftTrigger.whileHeld(new RunOnlyIndexer(m_indexer));
    // xBoxButtons[9].whenPressed(
    //     new SetTurretAbsoluteSetpointDegrees(m_turret, 0)
    //         .andThen(new SetTurretControlMode(m_turret, false)));
    // Climber
    xBoxButtons[9].whenPressed(new SetClimbState(m_climber, true, m_intake));
    xBoxRightTrigger.whileHeld(new RunIndexer(m_intake, m_indexer, m_flywheel, true));
    
    SmartDashboard.putData(new ResetOdometry(m_swerveDrive));
  }

  private void initializeAutoChooser() {
    m_autoChooser.addOption("Do Nothing", new DoNothing(m_swerveDrive));

    m_autoChooser.addOption("Drive Forward", new DriveForward(m_swerveDrive));

    m_autoChooser.addOption("TurnTurret", new TestAuto(m_swerveDrive, m_turret));

    m_autoChooser.addOption(
        "Three Ball Start",
        new ThreeBallAutoStart(
            m_swerveDrive, m_fieldSim, m_intake, m_indexer, m_flywheel, m_turret, m_vision));

    m_autoChooser.setDefaultOption(
        "Five ball",
        new FiveBallAuto(
            m_swerveDrive, m_fieldSim, m_intake, m_indexer, m_flywheel, m_turret, m_vision));

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

  public void periodic() {
    m_fieldSim.periodic();
  }

  public void disabledInit() {
    m_swerveDrive.setNeutralMode(NeutralMode.Coast);
  }

  public void disabledPeriodic() {}

  public void autonomousInit() {
    m_climber.setHoldPosition(m_climber.getElevatorClimbPosition());
    m_swerveDrive.setNeutralMode(NeutralMode.Brake);
  }

  public void autonomousPeriodic() {}

  public void teleopInit() {
    m_swerveDrive.setNeutralMode(NeutralMode.Brake);
    m_climber.setClimberNeutralMode(NeutralMode.Brake);
    m_climber.setHoldPosition(m_climber.getElevatorClimbPosition());
  }

  public void teleopPeriodic() {}

  public void simulationInit() {}

  public void simulationPeriodic() {}
}
