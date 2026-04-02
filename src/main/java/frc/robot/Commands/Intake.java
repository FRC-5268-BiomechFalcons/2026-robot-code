package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class Intake extends Command {
    // Subsystems
    IntakeSubsystem intakeSubsystem;
    ShooterSubsystem shooterSubsystem;
    private Timer agitatorTimer;
    // private final double agitatesPerSecond = 3;

    // Intake speed
    double speed;

    public Intake(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, double speed) {
        addRequirements(intakeSubsystem, shooterSubsystem);
        this.intakeSubsystem = intakeSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.speed = speed;
        this.agitatorTimer = new Timer();
    }

    /**
     * Runs the intake once the command starts.
     * The shooter runs at 1500 RPM to prevent jamming in between the indexer and shooter.
     */
    @Override
    public void initialize() {
        intakeSubsystem.intake(speed);
        shooterSubsystem.updateRPM(1700);
        shooterSubsystem.shoot();

        agitatorTimer.reset();
        agitatorTimer.start();
    }

    @Override
    public void execute() {
        // double power = Math.sin(2 * Math.PI * agitatesPerSecond * agitatorTimer.get());
        // intakeSubsystem.agitate(power);
    }

    /**
     * Stops all motors when the command ends.
     */
    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopMotors();
        shooterSubsystem.stopControl();
        // Resetting RPM back to default. 
        shooterSubsystem.updateRPM(3500);

        agitatorTimer.stop();
        agitatorTimer.reset();
    }

}