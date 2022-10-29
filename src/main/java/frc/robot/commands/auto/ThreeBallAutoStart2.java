package frc.robot.commands.auto;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.flywheel.SetAndHoldRpmSetpoint;
import frc.robot.commands.indexer.AutoRunIndexer;
import frc.robot.commands.intake.AutoRunIntake;
import frc.robot.commands.intake.IntakePiston;
import frc.robot.commands.swerve.SetSwerveNeutralMode;
import frc.robot.commands.swerve.SetSwerveOdometry;
import frc.robot.commands.turret.AutoUseVisionCorrection;
import frc.robot.commands.turret.SetTurretAbsoluteSetpointDegrees;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class ThreeBallAutoStart2 extends SequentialCommandGroup {
  public ThreeBallAutoStart2(
      SwerveDrive swerveDrive,
      Intake intake,
      Indexer indexer,
      Flywheel flywheel,
      Turret turret,
      Vision vision) {
    PathPlannerTrajectory trajectory1 =
        PathPlanner.loadPath(
            "ThreeBallAutoStart2-1", Units.feetToMeters(2), Units.feetToMeters(2), false);

    PathPlannerTrajectory trajectory2 =
        PathPlanner.loadPath(
            "ThreeBallAutoStart2-2", Units.feetToMeters(2), Units.feetToMeters(2), false);

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
        new SetSwerveOdometry(swerveDrive, trajectory1.getInitialPose()),
        new InstantCommand(() -> swerveDrive.setOdometry(trajectory1.getInitialPose())),
        // new SetOdometry(driveTrain, fieldSim, trajectory1.getInitialPose()), (probably don't )
        // Drive and shoot 2
        new ParallelCommandGroup(
            new SetTurretAbsoluteSetpointDegrees(turret, -5), new WaitCommand(.5)),
        new SetAndHoldRpmSetpoint(flywheel, vision, 1650),
        command1.andThen(() -> swerveDrive.drive(0, 0, 0, false, false)),
        new AutoUseVisionCorrection(turret, vision).withTimeout(1),
        new ConditionalCommand(new WaitCommand(0), new WaitCommand(0.5), flywheel::canShoot),
        new AutoRunIndexer(indexer, flywheel).withTimeout(1),
        new SetAndHoldRpmSetpoint(flywheel, vision, 1700),

        // Intake and shoot 3rd ball
        new ParallelDeadlineGroup(
            new AutoRunIntake(intake, indexer).withTimeout(1), new IntakePiston(intake, true)),
        new AutoUseVisionCorrection(turret, vision).withTimeout(1),
        new ConditionalCommand(new WaitCommand(0), new WaitCommand(0.5), flywheel::canShoot),
        new AutoRunIndexer(indexer, flywheel).withTimeout(1),
        new SetAndHoldRpmSetpoint(flywheel, vision, 700),
        new IntakePiston(intake, true),
        new ParallelDeadlineGroup(
            command2.andThen(() -> swerveDrive.drive(0, 0, 0, false, false)),
            new AutoRunIntake(intake, indexer),
            new SetTurretAbsoluteSetpointDegrees(turret, 60)),
        new AutoRunIntake(intake, indexer).withTimeout(1),
        new IntakePiston(intake, false),
        new ConditionalCommand(new WaitCommand(0), new WaitCommand(0.5), flywheel::canShoot),
        new AutoRunIndexer(indexer, flywheel).withTimeout(1),
        new SetAndHoldRpmSetpoint(flywheel, vision, 0));
        new SetSwerveNeutralMode(swerveDrive, NeutralMode.Brake)
        .andThen(() -> swerveDrive.drive(0, 0, 0, false, false));

    // stop intake (no intake stuff yet, need to implement)
    // shoot ball (no shooter stuff yet, need to implement)
    // run intake (no intake stuff yet, need to implement)

    // stop intake (no  intake stuff yet, need to implement)
    // shoot ball (no shooter stuff yet, need to implement)

    // TODO: Find out what this does
    // .andThen(() -> swerveDrive.setNeutralMode(NeutralMode.Brake))
    // .andThen(() -> swerveDrive.drive(0, 0, 0, false, false));
  }
}
