// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private Command autonomousCommand;

    private RobotContainer robotContainer;

    // Autonomous Chooser
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    private String lastAutoName = "";

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();

        // Adding all the autonomous options to the dashboard's auto selecter
        autoChooser.addOption("Drive Straight", robotContainer.driveStraightAuto());
        autoChooser.addOption("Shoot Preload Then Climb", robotContainer.shootThenClimbAuto());
        autoChooser.addOption("Shoot Preload", robotContainer.shootPreloadAuto());
        autoChooser.addOption("Left 1 Swipe + Climb", robotContainer.leftOneSwipeAndClimb());
        autoChooser.addOption("Left 2 Swipe + Climb", robotContainer.leftTwoSwipeAndClimb());
        autoChooser.addOption("Right 2 Swipe", robotContainer.rightTwoSwipe());

        SmartDashboard.putData(autoChooser);
        // robotContainer.driveSubsystem
        //         .setHeading(robotContainer.driveSubsystem.getPose().getRotation().getDegrees());

    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        /*
         * Runs the Scheduler. This is responsible for polling buttons, adding newly-scheduled
         * commands, running already-scheduled commands, removing finished or interrupted commands,
         * and running subsystem periodic() methods. This must be called from the robot's periodic
         * block in order for anything in the Command-based framework to work.
         */
        CommandScheduler.getInstance().run();
        SmartDashboard.putBoolean("Is Hub Active?", robotContainer.isHubActive());
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
        // Explicitly stop logging
        // If the user does not call stop(), then it's possible to lose the last few seconds of data
    }

    @Override
    public void disabledPeriodic() {
        Command selected = autoChooser.getSelected();

        if (selected != null && !selected.getName().equals(lastAutoName)) {
            lastAutoName = selected.getName();

            try {
                PathPlannerPath path = PathPlannerAuto.getPathGroupFromAutoFile(selected.getName()).get(0);

                Pose2d startPose = path.getStartingHolonomicPose().get();

                robotContainer.driveSubsystem.resetOdometry(startPose);
                robotContainer.driveSubsystem.resetQuest(startPose);

            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit() {
        autonomousCommand = autoChooser.getSelected();

        if (autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(autonomousCommand);

            // try {
            //     PathPlannerPath path = PathPlannerAuto
            //             .getPathGroupFromAutoFile(autoChooser.getSelected().getName()).get(0);

            //     Pose2d startPose = path.getStartingHolonomicPose().get();

            //     robotContainer.driveSubsystem.resetOdometry(startPose);
            //     robotContainer.driveSubsystem.resetQuest(startPose);

            // } catch (Exception e) {
            //     e.printStackTrace();
            // }
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        /*
         * This makes sure that the autonomous stops running when teleop starts running. If you want
         * the autonomous to continue until interrupted by another command, remove this line or
         * comment it out.
         */
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
        /*
         * COMPETITION TELEOPINIT RESET - Use this for comp.
         */
        // double estimatedRotDeg = robotContainer.driveSubsystem.getPose().getRotation().getDegrees();
        // robotContainer.driveSubsystem.setHeading(estimatedRotDeg);
        // robotContainer.driveSubsystem.setHeadingControlAngle(Math.toRadians(estimatedRotDeg));

        /*
         * For testing purposes, this resets the quest to a known position on the field. IMPORTANT:
         * Comment OUT lines 137-141 for COMPETITION.
         */
        Pose2d testingPose = new Pose2d(new Translation2d(3.5, 4), Rotation2d.fromDegrees(180));
        robotContainer.driveSubsystem.resetQuest(testingPose);
        robotContainer.driveSubsystem.resetOdometry(testingPose);
        robotContainer.driveSubsystem.setHeading(180);
        robotContainer.driveSubsystem.setHeadingControlAngle(Math.toRadians(180));

        // Changing the default RPM back to 3500 once teleop starts.
        robotContainer.shooterSubsystem.updateRPM(3500);
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();

        CommandScheduler.getInstance().schedule(robotContainer.autoShoot());
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
        CommandScheduler.getInstance().run();
    }
}