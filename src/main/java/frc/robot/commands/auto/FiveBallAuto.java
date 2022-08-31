package frc.robot.commands.auto;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.Constants.DriveTrain.DriveTrainNeutralMode;
// import frc.robot.commands.InterruptingCommand;
// import frc.robot.commands.driveTrain.CargoTrajectoryRameseteCommand;
// import frc.robot.commands.driveTrain.DriveToCargoTrajectory;
// import frc.robot.commands.driveTrain.SetDriveTrainNeutralMode;
// import frc.robot.commands.driveTrain.SetOdometry;
// import frc.robot.commands.flywheel.SetAndHoldRpmSetpoint;
// import frc.robot.commands.indexer.AutoRunIndexer;
// import frc.robot.commands.intake.AutoRunIntake;
// import frc.robot.commands.intake.AutoRunIntakeIndexer;
// import frc.robot.commands.intake.AutoRunIntakeInstant;
// import frc.robot.commands.intake.AutoRunIntakeOnly;
// import frc.robot.commands.intake.IntakePiston;
// import frc.robot.commands.simulation.SetSimTrajectory;
// import frc.robot.commands.simulation.SimulationShoot;
// import frc.robot.commands.turret.AutoUseVisionCorrection;
// import frc.robot.commands.turret.SetTurretAbsoluteSetpointDegrees;
// import frc.robot.simulation.FieldSim;
// import frc.robot.subsystems.DriveTrain;
// import frc.robot.subsystems.Flywheel;
// import frc.robot.subsystems.Indexer;
// import frc.robot.subsystems.Intake;
// import frc.robot.subsystems.Turret;
// import frc.robot.subsystems.Vision;
// import frc.vitruvianlib.utils.TrajectoryUtils;


public class FiveBallAuto extends SequentialCommandGroup {
  public FiveBallAuto(SwerveDrive swerveDrive) {
    PathPlannerTrajectory trajectory1 =
        PathPlanner.loadPath(
            "FiveBallAuto-1", Units.feetToMeters(2), Units.feetToMeters(2), false);
    PathPlannerTrajectory trajectory2 =
        PathPlanner.loadPath(
            "FiveBallAuto-2", Units.feetToMeters(2), Units.feetToMeters(2), false);
    PathPlannerTrajectory trajectory3 =
        PathPlanner.loadPath(
            "FiveBallAuto-3", Units.feetToMeters(2), Units.feetToMeters(2), false);
    PathPlannerTrajectory trajectory4 = 
        PathPlanner.loadPath(
            "FiveBallAuto-4", Units.feetToMeters(2), Units.feetToMeters(2), false);
    PPSwerveControllerCommand command1 =
        new PPSwerveControllerCommand(
            trajectory1,
            swerveDrive::getPoseMeters,
            Constants.SwerveDrive.kSwerveKinematics,
            swerveDrive.getXPidController(),
            swerveDrive.getYPidController(),
            swerveDrive.getThetaPidController(),
            swerveDrive::setSwerveModuleStatesAuto,
            swerveDrive);
    PPSwerveControllerCommand command2 =
        new PPSwerveControllerCommand(
            trajectory2,
            swerveDrive::getPoseMeters,
            Constants.SwerveDrive.kSwerveKinematics,
            swerveDrive.getXPidController(),
            swerveDrive.getYPidController(),
            swerveDrive.getThetaPidController(),
            swerveDrive::setSwerveModuleStatesAuto,
            swerveDrive);
    PPSwerveControllerCommand command3 = 
        new PPSwerveControllerCommand(
            trajectory3,
            swerveDrive::getPoseMeters,
            Constants.SwerveDrive.kSwerveKinematics,
            swerveDrive.getXPidController(),
            swerveDrive.getYPidController(),
            swerveDrive.getThetaPidController(),
            swerveDrive::setSwerveModuleStatesAuto,
            swerveDrive);
     PPSwerveControllerCommand command4 = 
        new PPSwerveControllerCommand(
            trajectory4,
            swerveDrive::getPoseMeters,
            Constants.SwerveDrive.kSwerveKinematics,
            swerveDrive.getXPidController(),
            swerveDrive.getYPidController(),
            swerveDrive.getThetaPidController(),
            swerveDrive::setSwerveModuleStatesAuto,
            swerveDrive);
    addCommands(
        // new InstantCommand(() -> swerveDrive.setOdometry(trajectory1.getInitialPose())),
        // new SetSimTrajectory(fieldSim, trajectory1, trajectory2, trajectory3),
        // new SetOdometry(driveTrain, fieldSim, trajectory1.getInitialPose()),
        // new SetDriveTrainNeutralMode(driveTrain, DriveTrainNeutralMode.BRAKE),
        
        // Path 1 + intake 1 cargo 
        // new IntakePiston(intake, true),
        // new SetAndHoldRpmSetpoint(flywheel, vision, 1600),
        // new ParallelDeadlineGroup(
            //new InterruptingCommand(
                 command1,//.andThen(() -> driveTrain.setMotorTankDrive(0, 0)),
            // new AutoRunIntakeIndexer(intake, indexer),
        //     new SetTurretAbsoluteSetpointDegrees(turret, 0)),
        // new IntakePiston(intake, false),

        // // Shoot 2
        // new AutoUseVisionCorrection(turret, vision).withTimeout(0.25),
        // new ConditionalCommand(
        //     new AutoRunIndexer(indexer, flywheel, 0.8).withTimeout(0.9),
        //     new SimulationShoot(fieldSim, true).withTimeout(0.9),
        //     RobotBase::isReal),
        
        // // Path 2 + intake 1 cargo
        // new IntakePiston(intake, true),
        // new SetAndHoldRpmSetpoint(flywheel, vision, 1700),
        // new ParallelDeadlineGroup(
        //     new InterruptingCommand(
                command2,//.andThen(() -> driveTrain.setMotorTankDrive(0, 0)),
        //     new AutoRunIntake(intake, indexer),
        //     // need this (SetTurret command) here?
        //     new SetTurretAbsoluteSetpointDegrees(turret, 20)),
        // new IntakePiston(intake, false),

        // // Shoot 1
        // new AutoUseVisionCorrection(turret, vision).withTimeout(0.25),
        // new ConditionalCommand(
        //     new AutoRunIndexer(indexer, flywheel, 0.8).withTimeout(0.7),
        //     new SimulationShoot(fieldSim, true).withTimeout(0.9),
        //     RobotBase::isReal),

        // // Path 3 + run intake + wait for human player (collect 2 cargo) --> already getting general range of hub
        // new SetAndHoldRpmSetpoint(flywheel, vision, 1700),
        // new SetTurretAbsoluteSetpointDegrees(turret, 30).withTimeout(0.25),
        // new IntakePiston(intake, true),
        // new AutoRunIntakeInstant(intake, indexer, true),
        //new ParallelDeadlineGroup(
                command3,//.andThen(() -> driveTrain.setMotorTankDrive(0, 0))
                // new AutoRunIntakeIndexer(intake, indexer).withTimeout(1)), // change this to 1.5? more time at the terminal (?)
        // new IntakePiston(intake, false),

        // // Path 4 + shoot 2
        command4//.andThen(() -> driveTrain.setMotorTankDrive(0, 0)),
        // new IntakePiston(intake, false),
        // new AutoUseVisionCorrection(turret, vision).withTimeout(0.75),
        // new ParallelDeadlineGroup(
        //     new ConditionalCommand(
        //         new AutoRunIndexer(indexer, flywheel, 0.80).withTimeout(5.0),
        //         new SimulationShoot(fieldSim, true).withTimeout(5.0),
        //         RobotBase::isReal),
        //     new AutoRunIntakeOnly(intake))
    );
  }
}
