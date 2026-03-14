// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;


public class DriveSubsystem extends SubsystemBase {
    // Create MAXSwerveModules
    private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(DriveConstants.kFrontLeftDrivingCanId,
        DriveConstants.kFrontLeftTurningCanId, DriveConstants.kFrontLeftChassisAngularOffset);
    private final MAXSwerveModule m_frontRight = new MAXSwerveModule(DriveConstants.kFrontRightDrivingCanId,
        DriveConstants.kFrontRightTurningCanId, DriveConstants.kFrontRightChassisAngularOffset);
    private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(DriveConstants.kRearLeftDrivingCanId,
        DriveConstants.kRearLeftTurningCanId, DriveConstants.kBackLeftChassisAngularOffset);
    private final MAXSwerveModule m_rearRight = new MAXSwerveModule(DriveConstants.kRearRightDrivingCanId,
        DriveConstants.kRearRightTurningCanId, DriveConstants.kBackRightChassisAngularOffset);

    // Pigeon IMU
    private final PigeonIMU m_gyro = new PigeonIMU(25);

    private Field2d field = new Field2d();
    QuestNav questNav = new QuestNav();

    boolean isResetting = false;

    private final SwerveDrivePoseEstimator m_odometry = new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics, Rotation2d.fromDegrees(m_gyro.getYaw()),
        new SwerveModulePosition[] { m_frontLeft.getPosition(), m_frontRight.getPosition(),
                m_rearLeft.getPosition(), m_rearRight.getPosition() },
        new Pose2d());

    // Percent of max speed, used for fine control
    private double m_speedModifier = 1.0;
    private Pose3d hub;
    private static final String shooterLimelight = "limelight-shooter";
    private static final String leftLimelight = "limelight-left";

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {
        // Usage reporting for MAXSwerve template
        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
        AutoBuilder.configure(this::getPose, this::resetOdometry, this::getRobotRelativeSpeeds,
                (speeds, feedsforwards) -> drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond,
                        speeds.omegaRadiansPerSecond, false),
                new PPHolonomicDriveController(new PIDConstants(0.2, 0.0, 0.0),
                    new PIDConstants(2, 0.0, 0.0)),
                getRobotConfig(), this::shouldFlipPath, this);

        if (shouldFlipPath()) {
            hub = FieldConstants.kHubTargetRed;
        } else {
            hub = FieldConstants.kHubTargetBlue;
        }
    }

    @Override
    public void periodic() {

        // Update the odometry in the periodic block
        m_odometry.update(Rotation2d.fromDegrees(m_gyro.getYaw()),
                new SwerveModulePosition[] { m_frontLeft.getPosition(), m_frontRight.getPosition(),
                        m_rearLeft.getPosition(), m_rearRight.getPosition() });

        // VISION POSE TRACKING - QUEST + 2 LIMELIGHTS
        questPoseTracking();
        limelightPoseTracking(shooterLimelight);
        // limelightPoseTracking(leftLimelight);

        field.setRobotPose(getPose());
        SmartDashboard.putData("Field", field);
        SmartDashboard.putData(field);
        SmartDashboard.putNumber("Distance to Hub", getDistanceToHub());
    }

    private void questPoseTracking() {
        questNav.commandPeriodic();

        if (isResetting) {
            PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();
            return;
        }
        PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();

        if (poseFrames.length > 0) {
            SmartDashboard.putBoolean("Quest State", true);
            PoseFrame last = poseFrames[poseFrames.length - 1];

            Pose3d questPose = last.questPose3d();
            Pose2d robotPose2d = questPose.transformBy(Constants.QuestConstants.ROBOT_TO_QUEST.inverse())
                    .toPose2d();

            /*
             * The higher the number, the less we trust the quest for that thing. So in this case,
             * we want to trust the quest for x and y, but we do not want to trust the quest on the
             * robot heading because the pigeon is probably more accurate
             */
            var questStdDevs = edu.wpi.first.math.VecBuilder.fill(0.05, // x meters
                    0.05, // y meters
                    1 // theta (radians)
            );

            m_odometry.addVisionMeasurement(robotPose2d, last.dataTimestamp(), questStdDevs);
        } else {
            SmartDashboard.putBoolean("Quest State", false);
        }
    }

    private void limelightPoseTracking(String limelight) {
        var alliance = DriverStation.getAlliance();
        boolean doRejectUpdate = false;
        LimelightHelpers.PoseEstimate estimatedPose = (alliance.isPresent() &&
            alliance.get() == DriverStation.Alliance.Red)
                    ? LimelightHelpers.getBotPoseEstimate_wpiRed(limelight)
                    : LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight);

        if (estimatedPose.tagCount == 1 && estimatedPose.rawFiducials.length == 1) {
            if (estimatedPose.rawFiducials[0].ambiguity > .7) {
                doRejectUpdate = true;
            }
            if (estimatedPose.rawFiducials[0].distToCamera > 3) {
                doRejectUpdate = true;
            }
        }
        if (estimatedPose.tagCount == 0) {
            doRejectUpdate = true;
        }

        if (!doRejectUpdate) {
            Pose2d pose = estimatedPose.pose;
            double timestamp = estimatedPose.timestampSeconds;
            var limelightStdDevs = edu.wpi.first.math.VecBuilder.fill(0.50, // x meters
                    0.50, // y meters
                    1 // theta (ignore)
            );

            m_odometry.addVisionMeasurement(pose, timestamp, limelightStdDevs);
        }

        /*
         * The quest is probably going to be more accurate than the limelight so the limelight
         * values are set a little higher than the quest. The higher the number, the less the pose
         * estimator trusts it. For the same reason as the quest, the pigeon should probably be
         */

    }

    public RobotConfig getRobotConfig() {
        try {
            return RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }

    public boolean shouldFlipPath() {
        var alliance = DriverStation.getAlliance();
        if (alliance.get() == DriverStation.Alliance.Red) {
            return true;
        } else {
            return false;
        }
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(m_frontLeft.getState(),
                m_frontRight.getState(), m_rearLeft.getState(), m_rearRight.getState());
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getEstimatedPosition();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {

        // Pigeon IMU
        m_odometry.resetPosition(Rotation2d.fromDegrees(m_gyro.getYaw()),
                new SwerveModulePosition[] { m_frontLeft.getPosition(), m_frontRight.getPosition(),
                        m_rearLeft.getPosition(), m_rearRight.getPosition() },
                pose);

        resetQuest(pose);
    }

    public void resetQuest(Pose2d pose) {
        Pose3d pose3d = new Pose3d(pose);
        questNav.setPose(pose3d.transformBy(Constants.QuestConstants.ROBOT_TO_QUEST));

        isResetting = true;
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        // Convert the commanded speeds into the correct units for the drivetrain
        double xSpeedDelivered = xSpeed * m_speedModifier * DriveConstants.kMaxSpeedMetersPerSecond;
        double ySpeedDelivered = ySpeed * m_speedModifier * DriveConstants.kMaxSpeedMetersPerSecond;
        double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

        // PIGEON IMU
        var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                        Rotation2d.fromDegrees(m_gyro.getYaw()))
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
                DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);

    }

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    public void setX() {
        m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_rearLeft.setDesiredState(desiredStates[2]);
        m_rearRight.setDesiredState(desiredStates[3]);
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        m_frontLeft.resetEncoders();
        m_rearLeft.resetEncoders();
        m_frontRight.resetEncoders();
        m_rearRight.resetEncoders();
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        m_gyro.setYaw(0);
        resetOdometry(getPose());
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        // return Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)).getDegrees();

        // PIGEON IMU
        return Rotation2d.fromDegrees(m_gyro.getYaw()).getDegrees();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        // return m_gyro.getRate(IMUAxis.kZ) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
        double[] ypr = new double[3];
        m_gyro.getRawGyro(ypr);
        return ypr[0] * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

    /**
     * Sets the speed modifier for the drive function.
     *
     * @param modifier The speed modifier (0.0 to 1.0)
     */
    public void setSpeedModifier(double modifier) {
        // Clamp between 0.0 and 1.0
        m_speedModifier = Math.max(0.0, Math.min(1.0, modifier));
    }

    public double getHubVectorAngle() {
        Pose3d pos = new Pose3d(getPose());
        Pose3d translation = pos.relativeTo(hub);
        return Math.toDegrees(Math.atan2(translation.getY(), translation.getX()));
    }

    public double findProjectileTrajectoryVelocity() {
        double heightDifference = Constants.FieldConstants.kHubHeight - Units.inchesToMeters(20);
        double g = 9.81; // Acceleration due to gravity in m/s^2
        double angleRadians = Math.toRadians(Constants.RobotConstants.kShooterAngle);
        Pose3d pos = new Pose3d(getPose());
        Pose3d translation = pos.relativeTo(hub);
        double d = Math.sqrt(Math.pow(translation.getX(), 2) + Math.pow(translation.getY(), 2)); // distance to hub

        // Using the projectile motion formula to calculate initial velocity
        double numerator = g * d * d;
        double denominator = 2 * (heightDifference - d * Math.tan(angleRadians)) *
            Math.pow(Math.cos(angleRadians), 2);

        if (denominator <= 0) {
            return 0; // No valid solution
        }

        return Math.sqrt(numerator / denominator);
    }

    public double getDistanceToHub() {
        Pose3d pos = new Pose3d(getPose());
        Pose3d translation = pos.relativeTo(hub);
        double d = Math.sqrt(Math.pow(translation.getX(), 2) + Math.pow(translation.getY(), 2));

        return d;
    }

    public Pose3d getHubPose() {
        return hub;
    }

    public Translation2d getFieldRelativeVelocity() {
        ChassisSpeeds robotRelativeSpeeds = getRobotRelativeSpeeds();
        ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds,
                Rotation2d.fromDegrees(getHeading()));

        return new Translation2d(fieldRelativeSpeeds.vxMetersPerSecond,
            fieldRelativeSpeeds.vyMetersPerSecond);
    }

}
