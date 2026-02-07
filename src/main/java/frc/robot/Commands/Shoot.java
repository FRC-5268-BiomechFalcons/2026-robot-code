package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class Shoot extends Command {
    ShooterSubsystem m_shooter;
    double goalRpm;
    IntakeSubsystem m_intakeSubsystem;
    double indexSpeed;
    boolean hitRPM;

    public Shoot(ShooterSubsystem m_shooter, IntakeSubsystem m_intakeSubsystem, double goalRpm,
            double indexSpeed) {
        addRequirements(m_shooter);
        this.m_shooter = m_shooter;
        this.goalRpm = goalRpm;
        this.m_intakeSubsystem = m_intakeSubsystem;
        this.indexSpeed = indexSpeed;
    }

    @Override
    public void initialize() {
        m_shooter.setSetpoint(goalRpm);
        hitRPM = false;
    }

    @Override
    public void execute() {

        if (m_shooter.getCurrentRPM() >= goalRpm - 25 && !hitRPM) {
            m_intakeSubsystem.index(indexSpeed);
            hitRPM = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stopControl();
        m_intakeSubsystem.stopMotors();

    }

}
