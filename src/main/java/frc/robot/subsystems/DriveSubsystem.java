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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
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

    // Field Widget for the dashboard
    private Field2d field = new Field2d();

    // Quest VR Headset 
    QuestNav questNav = new QuestNav();

    // Variable that tracks whether or not the quest has been resetted
    boolean isResetting = false;

    // Odometry Variable
    private final SwerveDrivePoseEstimator m_odometry = new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics, Rotation2d.fromDegrees(m_gyro.getYaw()),
        new SwerveModulePosition[] { m_frontLeft.getPosition(), m_frontRight.getPosition(),
                m_rearLeft.getPosition(), m_rearRight.getPosition() },
        new Pose2d());

    // Percent of max speed, used for fine control
    private double m_speedModifier = 1.0;

    // Limelight String Identifiers
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

    }

    @Override
    public void periodic() {

        // Update the odometry in the periodic block
        m_odometry.update(Rotation2d.fromDegrees(m_gyro.getYaw()),
                new SwerveModulePosition[] { m_frontLeft.getPosition(), m_frontRight.getPosition(),
                        m_rearLeft.getPosition(), m_rearRight.getPosition() });

        // VISION POSE TRACKING - QUEST + 2 LIMELIGHTS
        // questPoseTracking();
        limelightPoseTracking(shooterLimelight);
        limelightPoseTracking(leftLimelight);

        // Update the field widget with our new pose
        field.setRobotPose(getPose());

        // Update the dashboard
        SmartDashboard.putData("Field", field);
        SmartDashboard.putData(field);
        SmartDashboard.putNumber("Distance to Hub", getDistanceToHub());
        SmartDashboard.putNumber("Heading", getHeading());
    }

    /**
     * Updates the odometry's vision measurement using the quest's pose frames. 
     * This should be called periodically.
     * 
     */
    // private void questPoseTracking() {
    //     questNav.commandPeriodic();

    //     if (isResetting) {
    //         // Clear buffer and clear any remaining frames when the quest gets reset
    //         PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();
    //         if (poseFrames.length > 0) {
    //             isResetting = false;
    //         }
    //         return;
    //     }
    //     PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();

    //     if (poseFrames.length > 0) {
    //         SmartDashboard.putBoolean("Quest State", true);
    //         PoseFrame last = poseFrames[poseFrames.length - 1];

    //         Pose3d questPose = last.questPose3d();
    //         Pose2d robotPose2d = questPose.transformBy(Constants.QuestConstants.ROBOT_TO_QUEST.inverse())
    //                 .toPose2d();

    //         /*
    //          * The higher the number, the less we trust the quest for that thing. So in this case,
    //          * we want to trust the quest for x and y, but we do not want to trust the quest on the
    //          * robot heading because the pigeon is probably more accurate
    //          */
    //         var questStdDevs = edu.wpi.first.math.VecBuilder.fill(0.05, // x meters
    //                 0.05, // y meters
    //                 99999 // theta (radians)
    //         );

    //         m_odometry.addVisionMeasurement(last.questPose3d().toPose2d(), last.dataTimestamp(),
    //                 questStdDevs);
    //     } else {
    //         SmartDashboard.putBoolean("Quest State", false);
    //     }
    // }

    /**
     * Updates the odometry vision measurement using the Limelight's pose readings.
     * This should be called periodically.
     * 
     * @param limelight The limelight string identifier
     */
    private void limelightPoseTracking(String limelight) {
        var alliance = DriverStation.getAlliance();

        // Variable for whether or not we accept the limelight pose measurement
        boolean doRejectUpdate = false;

        // Receiving robot pose depending on which alliance we are in
        // LimelightHelpers.PoseEstimate estimatedPose = (alliance.get() == DriverStation.Alliance.Red)
        // ? LimelightHelpers.getBotPoseEstimate_wpiRed(limelight)
        //         : LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight);
        LimelightHelpers.PoseEstimate estimatedPose = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight);

        // Filtering the given pose measurement. Dismissing ambiguous or bad measurements
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
            // Updating the vision measurement with the given pose from the limelight
            Pose2d pose = estimatedPose.pose;
            double timestamp = estimatedPose.timestampSeconds;

            /*
             * The quest is probably going to be more accurate than the limelight so the limelight
             * values are set a little higher than the quest. The higher the number, the less the
             * pose estimator trusts it. For the same reason as the quest, the pigeon should
             * probably be
             */
            var limelightStdDevs = edu.wpi.first.math.VecBuilder.fill(0.50, // x meters
                    0.50, // y meters
                    1 // theta (ignore)
            );

            m_odometry.addVisionMeasurement(pose, timestamp, limelightStdDevs);
        }

    }

    /**
     * Fetches Pathplanner GUI settings. 
     * This should be applied in AutoBuilder for configuring pathplanner
     * 
     * @return Robot configuration settings from the pathplanner GUI.
     */
    public RobotConfig getRobotConfig() {
        try {
            return RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }

    /**
     * A helper method for configuring pathplanner. 
     * The default pathplanner setup is for the blue alliance, 
       so this checks whether or not we are red to decide to flip our autonomous routine.
     * This should be applied in AutoBuilder for configuring pathplanner.
     * 
     * @return boolean - whether or not we should flip the auto.
     */
    public boolean shouldFlipPath() {
        var alliance = DriverStation.getAlliance();
        if (alliance.get() == DriverStation.Alliance.Red) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Uses kinematics to convert our swerve module states to robot relative ChassisSpeeds.
     * 
     * @return Robot Relative ChassisSpeeds 
     */
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
        m_odometry.resetPosition(Rotation2d.fromDegrees(m_gyro.getYaw()),
                new SwerveModulePosition[] { m_frontLeft.getPosition(), m_frontRight.getPosition(),
                        m_rearLeft.getPosition(), m_rearRight.getPosition() },
                pose);

        // resetQuest(pose);
    }

    /**
     * Resets the quest's odometry. This should be called if you are resetting the robot's odometry.
     * 
     * @param pose The current Pose2d of the robot.
     */
    // public void resetQuest(Pose2d pose) {
    //     Pose3d pose3d = new Pose3d(pose);
    //     questNav.setPose(pose3d.transformBy(Constants.QuestConstants.ROBOT_TO_QUEST));

    //     isResetting = true;
    // }

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
        // return Rotation2d.fromDegrees(m_gyro.getYaw()).getDegrees();
        return m_odometry.getEstimatedPosition().getRotation().getDegrees();
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

    /**
     * Fetches the robot's field relative velocity by convering it from robot relative velocity.
     * 
     * @return a Translation2d of the robot's x and y field relative speeds
     */
    public Translation2d getFieldRelativeVelocity() {
        ChassisSpeeds robotRelativeSpeeds = getRobotRelativeSpeeds();
        ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds,
                Rotation2d.fromDegrees(getHeading()));

        return new Translation2d(fieldRelativeSpeeds.vxMetersPerSecond,
            fieldRelativeSpeeds.vyMetersPerSecond);
    }

    /**
     * This is exclusive for the 2026 FRC Game - Rebuilt.
     * Fetches the distance to the hub using the robot's pose and the hub pose.
     * 
     * @return distance, in meters, to the center of the hub
     */
    public double getDistanceToHub() {
        Pose3d pos = new Pose3d(getPose());
        Pose3d translation = pos.relativeTo(getHubPose());
        double d = Math.sqrt(Math.pow(translation.getX(), 2) + Math.pow(translation.getY(), 2));

        return d;
    }

    /**
     * This is exclusively for the 2026 FRC Game - Rebuilt
     * Fetches the pose of the hub, regardless of alliance.
     * 
     * @return a Pose3d of the hub's location on the field.
     */
    public Pose3d getHubPose() {
        if (shouldFlipPath()) {
            return FieldConstants.kHubTargetRed;
        } else {
            return FieldConstants.kHubTargetBlue;
        }
    }
}
