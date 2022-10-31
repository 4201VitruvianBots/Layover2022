package frc.robot.commands.auto;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.swerve.SetSwerveNeutralMode;
import frc.robot.subsystems.SwerveDrive;

public class DoNothing extends SequentialCommandGroup {
  public DoNothing(SwerveDrive swerveDrive) {
    addCommands(new SetSwerveNeutralMode(swerveDrive, NeutralMode.Brake));
  }
}
