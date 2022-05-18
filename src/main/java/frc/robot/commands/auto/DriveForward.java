package frc.robot.commands.auto;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

public class DriveForward extends SequentialCommandGroup {
  public DriveForward(SwerveDrive swerveDrive) {
    PathPlannerTrajectory trajectory =
        PathPlanner.loadPath("DriveForward", Units.feetToMeters(2), Units.feetToMeters(2), false);
    PPSwerveControllerCommand command =
        new PPSwerveControllerCommand(
            trajectory,
            swerveDrive::getPoseMeters,
            Constants.SwerveDrive.kSwerveKinematics,
            swerveDrive.getXPidController(),
            swerveDrive.getYPidController(),
            swerveDrive.getThetaPidController(),
            swerveDrive::setSwerveModuleStatesAuto,
            swerveDrive);
    addCommands(
        new InstantCommand(() -> swerveDrive.setOdometry(trajectory.getInitialPose())),
        command
            .andThen(() -> swerveDrive.setNeutralMode(NeutralMode.Brake))
            .andThen(() -> swerveDrive.drive(0, 0, 0, false, false)));
  }
}
