package frc.robot.commands.flywheel;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.Vision.CAMERA_POSITION;
import frc.robot.utils.FieldRelativeAccel;
import frc.robot.utils.FieldRelativeSpeed;
import frc.robot.utils.LinearInterpolationTable;
import frc.robot.subsystems.SwerveDrive; 
import frc.robot.subsystems.Vision; 
import frc.robot.subsystems.Flywheel; 
import frc.robot.subsystems.Turret;

public class SmartShooter extends CommandBase {
    private final Flywheel m_flywheel; 
    private final Turret m_turret;
    private final SwerveDrive m_swerveDrive; 
    private final boolean m_updatePose;
    private final XboxController m_driver;
    private final Vision m_vision; 
    private double m_wrongBallTime;
    private final Timer m_timer = new Timer();

    private static LinearInterpolationTable m_timeTable = Constants.Flywheel.kTimeTable;
    private static LinearInterpolationTable m_rpmTable = Constants.Flywheel.kRPMTable;

    public SmartShooter(Vision vision, Flywheel flywheel, Turret turret, SwerveDrive swervedrive, boolean updatePose,
        XboxController driver) {
        m_flywheel = flywheel; 
        m_turret = turret;
        m_swerveDrive = swervedrive; 
        m_updatePose = updatePose;
        m_driver = driver;
        m_vision = vision; 
        addRequirements(flywheel, turret); 
    }
    
    @Override
    public void initialize() {
        // m_turret.trackTarget(true); TODO: find substitute for this
        m_timer.reset();
        m_timer.start();
        SmartDashboard.putNumber("SetShotAdjust", 0);
        SmartDashboard.putBoolean("Adjust Shot?", false);
        m_wrongBallTime = Double.NEGATIVE_INFINITY;
    }

    @Override
    public void execute() {

        double currentTime = m_timer.get();

        SmartDashboard.putNumber("Current Time", currentTime);

        // SmartDashboard.putBoolean("Wrong Ball", wrongBall);

        SmartDashboard.putBoolean("Shooter Running", true);

        FieldRelativeSpeed robotVel = m_swerveDrive.getFieldRelativeSpeed();
        FieldRelativeAccel robotAccel = m_swerveDrive.getFieldRelativeAccel();

        Translation2d target = Constants.Vision.HUB_POSE.getTranslation(); //TODO: Double check 

        // if (currentTime <= m_wrongBallTime + 0.100) {
        //     target = GoalConstants.kWrongBallGoal;
        // }

        Translation2d robotToGoal = target.minus(m_swerveDrive.getPoseMeters().getTranslation());
        double dist = robotToGoal.getDistance(new Translation2d()) * 39.37;

        SmartDashboard.putNumber("Calculated (in)", dist);

        //double fixedShotTime = m_timeTable.getOutput(dist);
        double shotTime = m_timeTable.getOutput(dist);

        SmartDashboard.putNumber("Fixed Time", shotTime);

        Translation2d movingGoalLocation = new Translation2d();

        for(int i=0;i<5;i++){

            double virtualGoalX = target.getX()
                    - shotTime * (robotVel.vx + robotAccel.ax * Constants.Flywheel.kAccelCompFactor);
            double virtualGoalY = target.getY()
                    - shotTime * (robotVel.vy + robotAccel.ay * Constants.Flywheel.kAccelCompFactor);

            SmartDashboard.putNumber("Goal X", virtualGoalX);
            SmartDashboard.putNumber("Goal Y", virtualGoalY);

            Translation2d testGoalLocation = new Translation2d(virtualGoalX, virtualGoalY);

            Translation2d toTestGoal = testGoalLocation.minus(m_swerveDrive.getPoseMeters().getTranslation());

            double newShotTime = m_timeTable.getOutput(toTestGoal.getDistance(new Translation2d()) * 39.37);

            if(Math.abs(newShotTime-shotTime) <= 0.010){
                i=4;
            }
            
            if(i == 4){
                movingGoalLocation = testGoalLocation;
                SmartDashboard.putNumber("NewShotTime", newShotTime);
            }
            else{
                shotTime = newShotTime;
            }

        }

        double newDist = movingGoalLocation.minus(m_swerveDrive.getPoseMeters().getTranslation()).getDistance(new Translation2d()) * 39.37;

        SmartDashboard.putNumber("NewDist", newDist);

        m_turret.aimAtGoal(m_swerveDrive.getPoseMeters(), movingGoalLocation, false);

        if (SmartDashboard.getBoolean("Adjust Shot?", false)) {
            m_flywheel.run(m_rpmTable.getOutput(newDist) + SmartDashboard.getNumber("SetShotAdjust", 0));
        } else {
            m_flywheel.run(m_rpmTable.getOutput(newDist));

        }

        if (currentTime > 0.250 && m_vision.getValidTarget(CAMERA_POSITION.LIMELIGHT) && m_vision.getGoalTargetHorizontalDistance(CAMERA_POSITION.LIMELIGHT) >= 85.0) {
            double dL = m_vision.getGoalTargetHorizontalDistance(CAMERA_POSITION.LIMELIGHT) * 0.0254;
            double tR = m_swerveDrive.getHeadingRotation2d().getRadians();
            double tT = m_turret.getTurretAngleDegrees() - Math.PI;
            double tL = -1.0 * m_vision.getTargetXAngle(CAMERA_POSITION.LIMELIGHT);

            Pose2d pose = calcPoseFromVision(dL, tR, tT, tL, Constants.Vision.HUB_POSE.getTranslation());

            if (m_updatePose) {
                m_swerveDrive.setOdometry(pose); //was setPose; might need to change
            }

        }

    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("Shooter Running", false);
        // m_turret.trackTarget(false);
        m_turret.setPercentOutput(0);
        m_flywheel.setRPM(0);
        m_timer.stop();
        m_driver.setRumble(RumbleType.kLeftRumble, 0.0);
        m_driver.setRumble(RumbleType.kRightRumble, 0.0);
    }

    private Pose2d calcPoseFromVision(double dL, double tR, double tT, double tL, Translation2d goal) {
        double tG = tR + tT + tL;
        double rX = goal.getX() - dL * Math.cos(tG);
        double rY = goal.getY() - dL * Math.sin(tG);

        return new Pose2d(rX, rY, new Rotation2d(-tR));
    }

}
