package frc.robot.commands.auto;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.flywheel.SetAndHoldRpmSetpoint;
import frc.robot.commands.indexer.AutoRunIndexer;
import frc.robot.commands.intake.AutoRunIntake;
import frc.robot.commands.intake.AutoRunIntakeIndexer;
import frc.robot.commands.intake.AutoRunIntakeInstant;
import frc.robot.commands.intake.AutoRunIntakeOnly;
import frc.robot.commands.intake.IntakePiston;
import frc.robot.commands.swerve.SetSwerveNeutralMode;
import frc.robot.commands.turret.AutoUseVisionCorrection;
import frc.robot.commands.turret.SetTurretAbsoluteSetpointDegrees;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class FiveBallAuto extends SequentialCommandGroup {
  public FiveBallAuto(
      SwerveDrive swerveDrive,
      FieldSim fieldSim,
      Intake intake,
      Indexer indexer,
      Flywheel flywheel,
      Turret turret,
      Vision vision) {

    PathPlannerTrajectory trajectory1 =
        PathPlanner.loadPath(
            "FiveBallAuto-1", Units.feetToMeters(16), Units.feetToMeters(8), false);

    PathPlannerTrajectory trajectory2 =
        PathPlanner.loadPath(
            "FiveBallAuto-2", Units.feetToMeters(18), Units.feetToMeters(11), false);

    PathPlannerTrajectory trajectory3 =
        PathPlanner.loadPath(
            "FiveBallAuto-3", Units.feetToMeters(18), Units.feetToMeters(11), false);

    PathPlannerTrajectory trajectory4 =
        PathPlanner.loadPath(
            "FiveBallAuto-4", Units.feetToMeters(18), Units.feetToMeters(11), false);

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
        new InstantCommand(() -> swerveDrive.setOdometry(trajectory1.getInitialPose())),
        new SetSwerveNeutralMode(swerveDrive, NeutralMode.Brake),

        // Path 1 + intake 1 cargo
        // new AutoUseVisionCorrection(turret, vision).withTimeout(0.25),
        new IntakePiston(intake, true),
        new SetAndHoldRpmSetpoint(flywheel, vision, 1590),
        new SetTurretAbsoluteSetpointDegrees(turret, 7), // was 5
        new ParallelDeadlineGroup(
            // new InterruptingCommand(
            command1.andThen(() -> swerveDrive.drive(0, 0, 0, false, false)),
            // new DriveToCargoTrajectory(swerveDrive, vision),
            // () -> false),
            new AutoRunIntakeIndexer(intake, indexer)),
        new AutoUseVisionCorrection(turret, vision).withTimeout(0.5),
        new IntakePiston(intake, false),

        // // Shoot 3

        // new AutoUseVisionCorrection(turret, vision).withTimeout(0.25),
        new AutoRunIndexer(indexer, flywheel, 0.5).withTimeout(1.5),

        // // Path 2 + intake 1 cargo
        new IntakePiston(intake, true),
        new SetAndHoldRpmSetpoint(flywheel, vision, 1750),
        new ParallelDeadlineGroup(
            // new InterruptingCommand(
            command2.andThen(() -> swerveDrive.drive(0, 0, 0, false, false)),
            // new DriveToCargoTrajectory(driveTrain, vision),
            // () -> false),
            new AutoRunIntake(intake, indexer)),
        new IntakePiston(intake, false),

        // Shoot 1
        // need this (SetTurret command) here?
        // new SetTurretAbsoluteSetpointDegrees(turret, 5),
        new AutoUseVisionCorrection(turret, vision).withTimeout(0.5),
        new AutoRunIndexer(indexer, flywheel, 0.6).withTimeout(1),

        // Path 3 + run intake + wait for human player (collect 2 cargo) --> already getting
        // general range of hub
        new SetAndHoldRpmSetpoint(flywheel, vision, 1700),
        new IntakePiston(intake, true),
        new AutoRunIntakeInstant(intake, indexer, true),
        new ParallelDeadlineGroup(command3.andThen(() -> swerveDrive.drive(0, 0, 0, false, false))),
        new AutoRunIntakeIndexer(intake, indexer).withTimeout(1), // change this to 1.5? more
        new IntakePiston(intake, false),

        // // Path 4 + shoot 2
        new ParallelDeadlineGroup(
            command4.andThen(() -> swerveDrive.drive(0, 0, 0, false, false)),
            new SetAndHoldRpmSetpoint(flywheel, vision, 1850)),
        new SetTurretAbsoluteSetpointDegrees(turret, 0),
        new IntakePiston(intake, false),
        new AutoUseVisionCorrection(turret, vision).withTimeout(0.75),
        new ParallelDeadlineGroup(
            new AutoRunIndexer(indexer, flywheel, 0.4).withTimeout(5.0),
            new AutoRunIntakeOnly(intake)),
        new SetAndHoldRpmSetpoint(flywheel, vision, 0),
        new AutoUseVisionCorrection(turret, vision));
  }
}
