// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Commands.AutoRPM;
import frc.robot.Commands.AutoShoot;
// import frc.robot.Commands.Climb;
// import frc.robot.Commands.Hook;
import frc.robot.Commands.Index;
import frc.robot.Commands.Intake;
import frc.robot.Commands.Shoot;
import frc.robot.Commands.ShooterRevamp;
import frc.robot.Commands.StopMotors;
import frc.robot.Commands.UpdateRPM;
// import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.ShootOnTheFlyCalculator;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    public DriveSubsystem driveSubsystem = new DriveSubsystem();
    public ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    public IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    // public ClimbSubsystem climbSubsystem = new ClimbSubsystem();

    // The driver's controller
    CommandXboxController driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

    public ShootOnTheFlyCalculator sotfCalculator = new ShootOnTheFlyCalculator();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        driveSubsystem.setDefaultCommand(
                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.
                new RunCommand(() -> driveSubsystem.drive(
                        -MathUtil.applyDeadband(Math.pow(driverController.getLeftY(), 3),
                                OIConstants.kDriveDeadband),
                        -MathUtil.applyDeadband(Math.pow(driverController.getLeftX(), 3),
                                OIConstants.kDriveDeadband),
                        -MathUtil.applyDeadband(Math.pow(driverController.getRightX(), 3),
                                OIConstants.kDriveDeadband),
                        true),
                    driveSubsystem));

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

    /**
     * Registering commands for PathPlanner.
     */
    private void registerAutonomousCommands() {
        NamedCommands.registerCommand("Shooter", new ShooterRevamp(shooterSubsystem).withTimeout(2));
        NamedCommands.registerCommand("Index", new Index(intakeSubsystem).withTimeout(4));
        NamedCommands.registerCommand("Shoot Preload",
                new Shoot(shooterSubsystem, intakeSubsystem, RobotConstants.kShootingIndexSpeed)
                        .withTimeout(5));

        NamedCommands.registerCommand("Stop", new StopMotors(shooterSubsystem, intakeSubsystem));

        NamedCommands.registerCommand("Intake",
                new Intake(intakeSubsystem, shooterSubsystem, RobotConstants.kIntakeSpeed));

        NamedCommands.registerCommand("AutoShoot",
                new AutoShoot(shooterSubsystem, intakeSubsystem, driveSubsystem, sotfCalculator,
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
        driverController.rightStick()
                .onTrue(new InstantCommand(() -> driveSubsystem.zeroHeading(), driveSubsystem));

        // Passing
        driverController.leftTrigger().whileTrue(new SequentialCommandGroup(
            new ShooterRevamp(shooterSubsystem).withTimeout(1.1),
            new ParallelCommandGroup(new ShooterRevamp(shooterSubsystem), new Index(intakeSubsystem))));
        driverController.y().onTrue(new InstantCommand(() -> shooterSubsystem.updateRPM(4500)));
        driverController.a().onTrue(new InstantCommand(() -> shooterSubsystem.updateRPM(3500)));

        // Intake controls
        driverController.leftBumper()
                .whileTrue(new Intake(intakeSubsystem, shooterSubsystem, -RobotConstants.kIntakeSpeed));
        driverController.rightBumper()
                .toggleOnTrue(new Intake(intakeSubsystem, shooterSubsystem, RobotConstants.kIntakeSpeed));

        // Climber controls
        // driverController.back().and(driverController.x().negate()).and(driverController.b().negate())
        //         .whileTrue(new Hook(climbSubsystem, 0.2, Hook.Side.Both));
        // driverController.start().and(driverController.x().negate()).and(driverController.b().negate())
        //         .whileTrue(new Hook(climbSubsystem, -0.2, Hook.Side.Both));
        // driverController.back().and(driverController.x())
        //         .whileTrue(new Hook(climbSubsystem, 0.2, Hook.Side.Left));
        // driverController.start().and(driverController.x())
        //         .whileTrue(new Hook(climbSubsystem, -0.2, Hook.Side.Left));
        // driverController.back().and(driverController.b())
        //         .whileTrue(new Hook(climbSubsystem, 0.2, Hook.Side.Right));
        // driverController.start().and(driverController.b())
        //         .whileTrue(new Hook(climbSubsystem, -0.2, Hook.Side.Right));
        // LogicTriggers.without(driverController.x(), driverController.back())
        //         .whileTrue(new Climb(climbSubsystem, RobotConstants.kClimbSpeed));
        // LogicTriggers.without(driverController.b(), driverController.back())
        //         .whileTrue(new Climb(climbSubsystem, -RobotConstants.kClimbSpeed));

        // Manual RPM Increments - DPAD UP increases RPM Setpoint by 100, DPAD Down decreases RPM Setpoint by 100
        driverController.pov(0).onTrue(new UpdateRPM(shooterSubsystem, true));
        driverController.pov(180).onTrue(new UpdateRPM(shooterSubsystem, false));

        // Auto Shooting. Autoaims to the hub, then autoshoots with autonomously changing rpm. 
        driverController.rightTrigger()
                .whileTrue(new AutoShoot(shooterSubsystem, intakeSubsystem, driveSubsystem, sotfCalculator,
                    () -> -MathUtil.applyDeadband(Math.pow(driverController.getLeftY(), 3),
                            OIConstants.kDriveDeadband),
                    () -> -MathUtil.applyDeadband(Math.pow(driverController.getLeftX(), 3),
                            OIConstants.kDriveDeadband),
                    1, 0.02));

        driverController.rightTrigger()
                .whileTrue(new AutoShoot(shooterSubsystem, intakeSubsystem, driveSubsystem, sotfCalculator,
                    () -> -MathUtil.applyDeadband(Math.pow(driverController.getLeftY(), 3),
                            OIConstants.kDriveDeadband),
                    () -> -MathUtil.applyDeadband(Math.pow(driverController.getLeftX(), 3),
                            OIConstants.kDriveDeadband),
                    1, 0.02));

        driverController.x().whileTrue(
                new AutoRPM(shooterSubsystem, intakeSubsystem, driveSubsystem, sotfCalculator, 1, 0.02));

    }

    /*
     * AUTONOMOUS FUNCTIONS
     */
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

    public Command rightTwoSwipe() {
        try {
            return new PathPlannerAuto("RIGHT 2 Swipe");
        } catch (Exception e) {
            System.out.println("Error " + e);
            return Commands.none();
        }
    }

    /* Command for testing Auto rotation for field cal */

    public Command autoShoot() {
        return new AutoShoot(shooterSubsystem, intakeSubsystem, driveSubsystem, sotfCalculator,
            () -> -MathUtil.applyDeadband(Math.pow(driverController.getLeftY(), 3),
                    OIConstants.kDriveDeadband),
            () -> -MathUtil.applyDeadband(Math.pow(driverController.getLeftX(), 3),
                    OIConstants.kDriveDeadband),
            1, 0.02);
    }
}