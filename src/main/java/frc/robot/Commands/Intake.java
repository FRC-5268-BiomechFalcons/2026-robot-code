package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class Intake extends Command {
    IntakeSubsystem intakeSubsystem;
    double speed;
    ShooterSubsystem shooterSubsystem;

    public Intake(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, double speed) {
        addRequirements(intakeSubsystem, shooterSubsystem);
        this.intakeSubsystem = intakeSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        intakeSubsystem.intake(speed);
        shooterSubsystem.updateRPM(1500);
        shooterSubsystem.setSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopMotors();
        shooterSubsystem.stopControl();
    }

}