// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */


/**
 * Updates the RPM setpoint
 */
public class UpdateRPM extends InstantCommand {
    ShooterSubsystem shooterSubsystem;
    boolean isIncreasing;
    double rpm;

    /**
     * Constructor to increment RPM by 100.
     * @param shooterSubsystem
     * @param isIncreasing - Whether or not the RPM needs to be increased
     */
    public UpdateRPM(ShooterSubsystem shooterSubsystem, boolean isIncreasing) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shooterSubsystem);
        this.shooterSubsystem = shooterSubsystem;
        this.isIncreasing = isIncreasing;
        rpm = 0;
    }

    /**
     * Constructor to set the RPM to a specific value.
     * @param shooterSubsystem
     * @param rpm - New RPM setpoint value.
     */
    public UpdateRPM(ShooterSubsystem shooterSubsystem, double rpm) {
        addRequirements(shooterSubsystem);
        this.shooterSubsystem = shooterSubsystem;
        this.rpm = rpm;
    }

    /**
     * Called when the command is initially scheduled.
     * Updates the RPM to a new value.
     */
    @Override
    public void initialize() {
        if (rpm != 0) {
            shooterSubsystem.updateRPM(rpm);
        } else if (isIncreasing) {
            shooterSubsystem.updateRPM(shooterSubsystem.getUpdatingRPM() + 100);
        } else if (!isIncreasing) {
            shooterSubsystem.updateRPM(shooterSubsystem.getUpdatingRPM() - 100);
        }
    }
}
