package frc.robot.commands.auto;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

public class DriveForward extends SequentialCommandGroup {
  public DriveForward(SwerveDrive swerveDrive) {
    Trajectory trajectory =
        PathPlanner.loadPath("DriveForward", Units.feetToMeters(2), Units.feetToMeters(2), false);
    SwerveControllerCommand command =
        new SwerveControllerCommand(
            trajectory,
            swerveDrive::getPoseMeters,
            Constants.SwerveDrive.kSwerveKinematics,
            swerveDrive.getXPidController(),
            swerveDrive.getYPidController(),
            swerveDrive.getThetaPidController(),
            swerveDrive::setSwerveModuleStates,
            swerveDrive);
    addCommands(
        new InstantCommand(() -> swerveDrive.setOdometry(trajectory.getInitialPose())),
        command
            .andThen(() -> swerveDrive.setNeutralMode(NeutralMode.Brake))
            .andThen(() -> swerveDrive.drive(0, 0, 0, false, false)));
  }
}
