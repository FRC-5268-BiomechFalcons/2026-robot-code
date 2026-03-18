package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class Shoot extends Command {
    // Subsystems 
    private ShooterSubsystem shooter;
    private IntakeSubsystem intakeSubsystem;

    // Indexer speed.
    private double indexSpeed;

    // Whether or not the RPM setpoint has been hit
    private boolean hitRPM;

    // Timer for spinning up the shooter
    private Timer timer;

    public Shoot(ShooterSubsystem shooter, IntakeSubsystem intakeSubsystem, double indexSpeed) {
        addRequirements(shooter, intakeSubsystem);
        this.shooter = shooter;
        this.intakeSubsystem = intakeSubsystem;
        this.indexSpeed = indexSpeed;

        this.timer = new Timer();
    }

    /**
     * Run the shooter at the beginning of the command
     */
    @Override
    public void initialize() {
        shooter.shoot();
        timer.reset();
        timer.start();
        hitRPM = false;
    }

    /**
     * Once the shooter hits the setpoint, start indexing. 
     */
    @Override
    public void execute() {
        if (shooter.hitRPMSetpoint() && !hitRPM && timer.hasElapsed(1.2)) {
            intakeSubsystem.index(indexSpeed);
            hitRPM = true;
        }
    }

    /**
     * Stop all motors once the command ends.
     */
    @Override
    public void end(boolean interrupted) {
        shooter.stopControl();
        intakeSubsystem.stopMotors();
        timer.stop();
        timer.reset();
    }

}