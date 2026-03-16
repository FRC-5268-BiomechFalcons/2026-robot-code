package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class Intake extends Command {
    // Subsystems
    IntakeSubsystem intakeSubsystem;
    ShooterSubsystem shooterSubsystem;

    // Intake speed
    double speed;

    public Intake(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, double speed) {
        addRequirements(intakeSubsystem, shooterSubsystem);
        this.intakeSubsystem = intakeSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.speed = speed;
    }

    /**
     * Runs the intake once the command starts.
     * The shooter runs at 1500 RPM to prevent jamming in between the indexer and shooter.
     */
    @Override
    public void initialize() {
        intakeSubsystem.intake(speed);
        shooterSubsystem.updateRPM(1500);
        shooterSubsystem.shoot();
    }

    /**
     * Stops all motors when the command ends.
     */
    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopMotors();
        shooterSubsystem.stopControl();
    }

}