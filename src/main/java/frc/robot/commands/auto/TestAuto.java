package frc.robot.commands.auto;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.swerve.SetSwerveNeutralMode;
import frc.robot.commands.turret.SetTurretAbsoluteSetpointDegrees;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Turret;

public class TestAuto extends SequentialCommandGroup {
  public TestAuto(SwerveDrive swerveDrive, Turret turret) {
    addCommands(
        new SetSwerveNeutralMode(swerveDrive, NeutralMode.Brake),
        new SetTurretAbsoluteSetpointDegrees(turret, 30));
  }
}
