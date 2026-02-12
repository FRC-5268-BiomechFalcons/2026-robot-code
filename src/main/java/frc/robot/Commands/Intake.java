package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.IntakeSubsystem;


public class Intake extends Command {
    IntakeSubsystem intakeSubsystem;
    double speed;

    public Intake(IntakeSubsystem intakeSubsystem, double speed) {
        addRequirements(intakeSubsystem);
        this.intakeSubsystem = intakeSubsystem;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        intakeSubsystem.intake(speed);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopMotors();
    }

}
