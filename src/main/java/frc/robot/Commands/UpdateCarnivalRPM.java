// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShooterSubsystem;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class UpdateCarnivalRPM extends InstantCommand {
    ShooterSubsystem shooterSubsystem;
    boolean isIncreasing;

    public UpdateCarnivalRPM(ShooterSubsystem shooterSubsystem, boolean isIncreasing) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.shooterSubsystem = shooterSubsystem;
        this.isIncreasing = isIncreasing;

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (isIncreasing) {
            shooterSubsystem.updateCarnivalRPM(shooterSubsystem.getCarnivalRPM() + 50);
        } else {
            shooterSubsystem.updateCarnivalRPM(shooterSubsystem.getCarnivalRPM() - 50);
        }
    }
}
