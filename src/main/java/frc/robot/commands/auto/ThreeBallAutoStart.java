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

public class ThreeBallAutoStart extends SequentialCommandGroup {
  public ThreeBallAutoStart(
      SwerveDrive swerveDrive,
      FieldSim fieldSim,
      Intake intake,
      Indexer indexer,
      Flywheel flywheel,
      Turret turret,
      Vision vision) {

    PathPlannerTrajectory trajectory1 =
        PathPlanner.loadPath(
            "ThreeBallAutoStart-1", Units.feetToMeters(6), Units.feetToMeters(2), false);

    PathPlannerTrajectory trajectory2 =
        PathPlanner.loadPath(
            "ThreeBallAutoStart-2", Units.feetToMeters(6), Units.feetToMeters(2), false);

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

    addCommands(
        new InstantCommand(() -> swerveDrive.setOdometry(trajectory1.getInitialPose())),
        new SetSwerveNeutralMode(swerveDrive, NeutralMode.Brake),

        // Path 1 + intake 1 cargo
        new IntakePiston(intake, false),
        new SetAndHoldRpmSetpoint(flywheel, vision, 1650),
        new ParallelDeadlineGroup(
            command1.andThen(() -> swerveDrive.drive(0, 0, 0, false, false)),
            new AutoRunIntakeIndexer(intake, indexer),
            new SetTurretAbsoluteSetpointDegrees(turret, 0)),
        new AutoUseVisionCorrection(turret, vision).withTimeout(0.5),
        // // Shoot 2
        new IntakePiston(intake, true),
        new IntakePiston(intake, false),
        new AutoRunIndexer(indexer, flywheel, 0.6).withTimeout(2),

        // // Path 2 + intake 1 cargo
        new IntakePiston(intake, true),
        new SetAndHoldRpmSetpoint(flywheel, vision, 1200),
        new ParallelDeadlineGroup(
            // new InterruptingCommand(
            command2.andThen(() -> swerveDrive.drive(0, 0, 0, false, false)),
            // new DriveToCargoTrajectory(driveTrain, vision),
            // () -> false),
            new AutoRunIntake(intake, indexer),
            // need this (SetTurret command) here?
            new SetTurretAbsoluteSetpointDegrees(turret, 20)),
        new IntakePiston(intake, false),

        // Shoot 1
        new ParallelDeadlineGroup(
            new AutoRunIndexer(indexer, flywheel, 0.6).withTimeout(5.0),
            new AutoRunIntakeOnly(intake)));
  }
}
