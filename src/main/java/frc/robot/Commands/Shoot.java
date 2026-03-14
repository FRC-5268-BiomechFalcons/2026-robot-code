package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class Shoot extends Command {
    ShooterSubsystem shooter;
    IntakeSubsystem intakeSubsystem;
    double indexSpeed;
    boolean hitRPM;

    public Shoot(ShooterSubsystem shooter, IntakeSubsystem intakeSubsystem, double indexSpeed) {
        addRequirements(shooter, intakeSubsystem);
        this.shooter = shooter;
        this.intakeSubsystem = intakeSubsystem;
        this.indexSpeed = indexSpeed;
    }

    @Override
    public void initialize() {
        shooter.setSetpoint();
        hitRPM = false;
    }

    @Override
    public void execute() {

        if (shooter.hitRPMSetpoint() && !hitRPM) {
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