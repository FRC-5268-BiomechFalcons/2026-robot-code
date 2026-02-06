// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Commands.Index;
import frc.robot.Commands.Intake;
import frc.robot.Commands.Shoot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


// import c
/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();

    // The driver's controller
    CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
    ShooterSubsystem m_shooter = new ShooterSubsystem();
    IntakeSubsystem m_intake = new IntakeSubsystem();
    // General Motors
    // private final SparkMax m_intakeMotor = new SparkMax(8, MotorType.kBrushless);
    // private final SparkMax m_indexerMotor = new SparkMax(9, MotorType.kBrushless);
    // private final TalonFX m_shooterMotor = new TalonFX(7);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        m_robotDrive.setDefaultCommand(
                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.
                new RunCommand(() -> m_robotDrive.drive(
                        -MathUtil.applyDeadband(Math.pow(m_driverController.getLeftY(), 3),
                                OIConstants.kDriveDeadband),
                        -MathUtil.applyDeadband(Math.pow(m_driverController.getLeftX(), 3),
                                OIConstants.kDriveDeadband),
                        -MathUtil.applyDeadband(Math.pow(m_driverController.getRightX(), 3),
                                OIConstants.kDriveDeadband),
                        true),
                    m_robotDrive));
    }

    /*
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        // Zero the heading when the right stick is pressed
        m_driverController.rightStick()
                .onTrue(new RunCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));

        // Reduce the speed of the robot when the left trigger is held
        m_driverController.leftTrigger()
                .onTrue(new RunCommand(() -> m_robotDrive.setSpeedModifier(0.5), m_robotDrive))
                .onFalse(new RunCommand(() -> m_robotDrive.setSpeedModifier(1.0), m_robotDrive));

        // Intake controls

        m_driverController.rightBumper().whileTrue(new Intake(m_intake, 0.6));
        m_driverController.leftBumper().whileTrue(new Intake(m_intake, -0.6));

        // Shooter controls 
        m_driverController.x().whileTrue(new Shoot(m_shooter, m_intake, RobotConstants.kShooterVelocity, .6));

        m_driverController.y()
                .whileTrue(new Shoot(m_shooter, m_intake, -RobotConstants.kShooterVelocity, .6));

        m_driverController.rightTrigger()
                .whileTrue(new Shoot(m_shooter, m_intake, RobotConstants.kShooterVelocity, .6));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                    // Add kinematics to ensure max speed is actually obeyed
                    .setKinematics(DriveConstants.kDriveKinematics);

        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, new Rotation2d(0)), config);

        var thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0,
            AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(exampleTrajectory,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0), thetaController,
            m_robotDrive::setModuleStates, m_robotDrive);

        // Reset odometry to the starting pose of the trajectory.
        m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
    }
}
