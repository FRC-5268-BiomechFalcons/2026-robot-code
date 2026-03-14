// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Commands.Climb;
import frc.robot.Commands.Hook;
import frc.robot.Commands.Index;
import frc.robot.Commands.Intake;
import frc.robot.Commands.Shoot;
import frc.robot.Commands.AutoShoot;
import frc.robot.Commands.ShooterRevamp;
import frc.robot.Commands.StopMotors;
import frc.robot.Commands.UpdateRPM;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.LogicTriggers;
import frc.robot.util.ShootOnTheFlyCalculator;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    public final DriveSubsystem robotDrive = new DriveSubsystem();

    // The driver's controller
    CommandXboxController driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

    // Subsystems
    public ShooterSubsystem shooter = new ShooterSubsystem();
    public IntakeSubsystem intake = new IntakeSubsystem();
    public ClimbSubsystem climb = new ClimbSubsystem();

    public ShootOnTheFlyCalculator sotfCalculator = new ShootOnTheFlyCalculator();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        robotDrive.setDefaultCommand(
                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.
                new RunCommand(() -> robotDrive.drive(
                        -MathUtil.applyDeadband(Math.pow(driverController.getLeftY(), 3),
                                OIConstants.kDriveDeadband),
                        -MathUtil.applyDeadband(Math.pow(driverController.getLeftX(), 3),
                                OIConstants.kDriveDeadband),
                        -MathUtil.applyDeadband(Math.pow(driverController.getRightX(), 3),
                                OIConstants.kDriveDeadband),
                        true),
                    robotDrive));

        registerAutonomousCommands();

        // Configuring RPM Table
        sotfCalculator.addTableEntry(1.78, 2700, 0, 1.04);
        sotfCalculator.addTableEntry(2.03, 2850, 0, 0.66);
        sotfCalculator.addTableEntry(2.32, 2950, 0, 0.83);
        sotfCalculator.addTableEntry(2.68, 3000, 0, 0.88);
        sotfCalculator.addTableEntry(3.08, 3250, 0, 0.81);
        sotfCalculator.addTableEntry(3.34, 3400, 0, 1.01);
        sotfCalculator.addTableEntry(3.85, 3800, 0, 1.04);
        sotfCalculator.addTableEntry(4.05, 3850, 0, 1.05);
    }

    private void registerAutonomousCommands() {
        NamedCommands.registerCommand("Shooter", new ShooterRevamp(shooter).withTimeout(2));
        NamedCommands.registerCommand("Index", new Index(intake).withTimeout(4));
        NamedCommands.registerCommand("Shoot Preload",
                new Shoot(shooter, intake, RobotConstants.kShootingIndexSpeed).withTimeout(5));

        NamedCommands.registerCommand("Stop", new StopMotors(shooter, intake));

        NamedCommands.registerCommand("Intake", new Intake(intake, shooter, RobotConstants.kIntakeSpeed));

        NamedCommands.registerCommand("AutoShoot",
                new AutoShoot(shooter, intake, robotDrive, sotfCalculator,
                    () -> -MathUtil.applyDeadband(Math.pow(driverController.getLeftY(), 3),
                            OIConstants.kDriveDeadband),
                    () -> -MathUtil.applyDeadband(Math.pow(driverController.getLeftX(), 3),
                            OIConstants.kDriveDeadband),
                    0.5, 0.02));
    }

    /*
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        // Zero the heading when the right stick is pressed - Reset Field Relative.
        driverController.rightStick().onTrue(new InstantCommand(() -> robotDrive.zeroHeading(), robotDrive));

        // Passing
        driverController.leftTrigger()
                .whileTrue(new SequentialCommandGroup(new ShooterRevamp(shooter).withTimeout(1.1),
                    new ParallelCommandGroup(new ShooterRevamp(shooter), new Index(intake))));
        driverController.y().onTrue(new InstantCommand(() -> shooter.updateRPM(4500)));
        driverController.a().onTrue(new InstantCommand(() -> shooter.updateRPM(3500)));

        // Intake controls
        driverController.leftBumper().whileTrue(new Intake(intake, shooter, -RobotConstants.kIntakeSpeed));
        driverController.rightBumper().toggleOnTrue(new Intake(intake, shooter, RobotConstants.kIntakeSpeed));

        // Climber controls
        driverController.back().and(driverController.x().negate()).and(driverController.b().negate())
                .whileTrue(new Hook(climb, 0.2, Hook.Side.Both));
        driverController.start().and(driverController.x().negate()).and(driverController.b().negate())
                .whileTrue(new Hook(climb, -0.2, Hook.Side.Both));
        driverController.back().and(driverController.x()).whileTrue(new Hook(climb, 0.2, Hook.Side.Left));
        driverController.start().and(driverController.x()).whileTrue(new Hook(climb, -0.2, Hook.Side.Left));
        driverController.back().and(driverController.b()).whileTrue(new Hook(climb, 0.2, Hook.Side.Right));
        driverController.start().and(driverController.b()).whileTrue(new Hook(climb, -0.2, Hook.Side.Right));
        LogicTriggers.without(driverController.x(), driverController.back())
                .whileTrue(new Climb(climb, RobotConstants.kClimbSpeed));
        LogicTriggers.without(driverController.b(), driverController.back())
                .whileTrue(new Climb(climb, -RobotConstants.kClimbSpeed));

        // Manual RPM Increments - DPAD UP increases RPM Setpoint by 100, DPAD Down decreases RPM Setpoint by 100
        driverController.pov(0).onTrue(new UpdateRPM(shooter, true));
        driverController.pov(180).onTrue(new UpdateRPM(shooter, false));

        // Auto Shooting. Autoaims to the hub, then autoshoots with autonomously changing rpm. 
        driverController.rightTrigger()
                .whileTrue(new AutoShoot(shooter, intake, robotDrive, sotfCalculator,
                    () -> -MathUtil.applyDeadband(Math.pow(driverController.getLeftY(), 3),
                            OIConstants.kDriveDeadband),
                    () -> -MathUtil.applyDeadband(Math.pow(driverController.getLeftX(), 3),
                            OIConstants.kDriveDeadband),
                    1, 0.02));

    }

    public Command driveStraightAuto() {
        try {
            return new PathPlannerAuto("Drive Straight");
        } catch (Exception e) {
            System.out.println("Error " + e);
            return Commands.none();
        }
    }

    public Command leftOneSwipeAndClimb() {
        try {
            return new PathPlannerAuto("LEFT 1 Swipe + Climb");
        } catch (Exception e) {
            System.out.println("Error " + e);
            return Commands.none();
        }
    }

    public Command leftTwoSwipeAndClimb() {
        try {
            return new PathPlannerAuto("LEFT 2 Swipe Then Climb");
        } catch (Exception e) {
            System.out.println("Error " + e);
            return Commands.none();
        }
    }

    public Command shootPreloadAuto() {
        try {
            return new PathPlannerAuto("Shoot Preload");
        } catch (Exception e) {
            System.out.println("Error " + e);
            return Commands.none();
        }
    }

    public Command shootThenClimbAuto() {
        try {
            return new PathPlannerAuto("Shoot Preload Then Climb");
        } catch (Exception e) {
            System.out.println("Error " + e);
            return Commands.none();
        }
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
            robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0), thetaController,
            robotDrive::setModuleStates, robotDrive);

        // Reset odometry to the starting pose of the trajectory.
        robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return swerveControllerCommand.andThen(() -> robotDrive.drive(0, 0, 0, false));
    }
}