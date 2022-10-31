package frc.robot.commands.auto;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.indexer.AutoRunIndexer;
import frc.robot.commands.swerve.SetSwerveNeutralMode;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.SwerveDrive;

public class TestAuto extends SequentialCommandGroup {
  public TestAuto(SwerveDrive swerveDrive, Indexer indexer, Flywheel flywheel) {
    addCommands(
        new SetSwerveNeutralMode(swerveDrive, NeutralMode.Brake),
        new AutoRunIndexer(indexer, flywheel, 0.6));
  }
}
