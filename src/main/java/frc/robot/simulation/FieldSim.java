// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulation;

import static frc.robot.Constants.SwerveDrive.kModuleTranslations;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveDrive.ModulePosition;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;
import frc.robot.utils.ModuleMap;
import java.util.Map;

public class FieldSim {
  private final SwerveDrive m_swerveDrive;
  private final Vision m_vision;

  private final Field2d m_field2d = new Field2d();

  private final Map<ModulePosition, Pose2d> m_swerveModulePoses =
      ModuleMap.of(new Pose2d(), new Pose2d(), new Pose2d(), new Pose2d());

  public FieldSim(SwerveDrive swerveDrive, Vision vision) {
    m_swerveDrive = swerveDrive;
    m_vision = vision;
  }

  public void initSim() {}

  public Field2d getField2d() {
    return m_field2d;
  }

  private void updateRobotPoses() {
    m_field2d.setRobotPose(m_swerveDrive.getPoseMeters());

    for (ModulePosition i : ModulePosition.values()) {
      Translation2d updatedPositions =
          kModuleTranslations
              .get(i)
              .rotateBy(m_swerveDrive.getPoseMeters().getRotation())
              .plus(m_swerveDrive.getPoseMeters().getTranslation());
      m_swerveModulePoses.put(
          i,
          new Pose2d(
              updatedPositions,
              m_swerveDrive
                  .getSwerveModule(i)
                  .getHeadingRotation2d()
                  .plus(m_swerveDrive.getHeadingRotation2d())));
    }

    m_field2d
        .getObject("Swerve Modules")
        .setPoses(ModuleMap.orderedValues(m_swerveModulePoses, new Pose2d[0]));
    try {
      int[] pruneIds = {0, 1, 2, 3, 4};
      for (int i = 0; i < m_vision.getCameraRobotPoseIDs().length; i++) {
        var translation = m_vision.getCameraRobotPoses()[i].getTranslation();
        translation.rotateBy(new Rotation2d(90));

        m_field2d.getObject("Pose " + m_vision.getCameraRobotPoseIDs()[i]).setPose(
                new Pose2d(translation, new Rotation2d()));
        pruneIds[m_vision.getCameraRobotPoseIDs()[i]] = 0;
      }
      for (int i = 0; i < pruneIds.length; i++) {
        m_field2d.getObject("Pose " + pruneIds[i]).setPose(new Pose2d(-1, -1, new Rotation2d()));
      }

      pruneIds = new int[]{0, 1, 2, 3, 4};
      for (int i = 0; i < m_vision.getPnpCameraRobotPoseIDs().length; i++) {
        var translation = m_vision.getPnpCameraRobotPoses()[i].getTranslation();
        translation.rotateBy(new Rotation2d(90));

        m_field2d.getObject("PNP Pose " + m_vision.getPnpCameraRobotPoseIDs()[i]).setPose(
                new Pose2d(translation, new Rotation2d()));
        pruneIds[m_vision.getPnpCameraRobotPoseIDs()[i]] = 0;
      }
      for (int i = 0; i < pruneIds.length; i++) {
        m_field2d.getObject("PNP Pose " + pruneIds[i]).setPose(new Pose2d(-1, -1, new Rotation2d()));
      }

      m_field2d.getObject("Avg Pose").setPose(m_vision.getAvgPose());
    } catch (Exception e) {

    }
  }

  public void periodic() {
    updateRobotPoses();

    if (RobotBase.isSimulation()) simulationPeriodic();

    SmartDashboard.putData("Field2d", m_field2d);
  }

  public void simulationPeriodic() {}
}
