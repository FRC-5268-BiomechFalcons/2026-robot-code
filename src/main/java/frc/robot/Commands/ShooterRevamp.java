// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterRevamp extends Command {
    // Shooter subsystem to run the shooter
    ShooterSubsystem shooterSubsystem;

    /** Creates a new ShooterRevamp. */
    public ShooterRevamp(ShooterSubsystem shooterSubsystem) {
        addRequirements(shooterSubsystem);
        this.shooterSubsystem = shooterSubsystem;
        // Use addRequirements() here to declare subsystem dependencies.
    }

    /**
     * Called when the command is initally scheduled.
     * Run the shooter at the start of the command
     */
    @Override
    public void initialize() {
        shooterSubsystem.shoot();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    /**
     * Called once the command ends or is interrupted.
     * Stop the shooter at the end of the command
     */
    @Override
    public void end(boolean interrupted) {
        // 
        shooterSubsystem.stopControl();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
