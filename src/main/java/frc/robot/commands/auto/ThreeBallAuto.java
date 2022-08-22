package frc.robot.commands.auto;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

public class ThreeBallAuto extends SequentialCommandGroup {
  public ThreeBallAuto(SwerveDrive swerveDrive) {
    PathPlannerTrajectory trajectory1 =
        PathPlanner.loadPath(
            "ThreeBallAuto-1", Units.feetToMeters(2), Units.feetToMeters(2), false);
    PathPlannerTrajectory trajectory2 =
        PathPlanner.loadPath(
            "ThreeBallAuto-2", Units.feetToMeters(2), Units.feetToMeters(2), false);
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
        // new InstantCommand(() -> swerveDrive.setOdometry(trajectory1.getInitialPose())),
        // new SetSimTrajectory(fieldSim, trajectory1, trajectory2, trajectory3),
        // // new SetOdometry(driveTrain, fieldSim, trajectory1.getInitialPose()), (probably don't )
        // new SetDriveTrainNeutralMode(driveTrain, DriveTrainNeutralMode.BRAKE),
        // new IntakePiston(intake, true),
        // new ParallelCommandGroup(
        //     new SetTurretAbsoluteSetpointDegrees(turret, -5), new WaitCommand(.5)),
        // new SetAndHoldRpmSetpoint(flywheel, vision, 2400),
        // new ParallelDeadlineGroup(
        //     command1.andThen(() -> driveTrain.setMotorTankDrive(0, 0)),
        //     new AutoRunIntake(intake, indexer)),

        // stop intake (no intake stuff yet, need to implement)
        // shoot ball (no shooter stuff yet, need to implement)
        // run intake (no intake stuff yet, need to implement)
        command2
            // stop intake (no intake stuff yet, need to implement)
            // shoot ball (no shooter stuff yet, need to implement)
            .andThen(() -> swerveDrive.setNeutralMode(NeutralMode.Brake))
            .andThen(() -> swerveDrive.drive(0, 0, 0, false, false)));
  }
}
