package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class Shoot extends Command {
    ShooterSubsystem shooter;
    double goalRpm;
    IntakeSubsystem intakeSubsystem;
    double indexSpeed;
    boolean hitRPM;

    public Shoot(ShooterSubsystem shooter, IntakeSubsystem intakeSubsystem, double goalRpm,
            double indexSpeed) {
        addRequirements(shooter, intakeSubsystem);
        this.shooter = shooter;
        this.intakeSubsystem = intakeSubsystem;
        this.goalRpm = goalRpm;
        this.indexSpeed = indexSpeed;
    }

    @Override
    public void initialize() {
        shooter.setSetpoint(goalRpm);
        hitRPM = false;
    }

    @Override
    public void execute() {

        if (shooter.getCurrentRPM() >= goalRpm - 25 && !hitRPM) {
            intakeSubsystem.index(indexSpeed);
            hitRPM = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopControl();
        intakeSubsystem.stopMotors();

    }

}
