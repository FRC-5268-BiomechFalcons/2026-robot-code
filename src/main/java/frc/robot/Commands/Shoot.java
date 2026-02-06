package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class Shoot extends Command {
    ShooterSubsystem m_shooter;
    double goalRpm;
    IntakeSubsystem m_intakeSubsystem;
    double indexSpeed;

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

    }

    @Override
    public void execute() {
        if (m_shooter.getCurrentRPM() >= goalRpm - 50) {
            m_intakeSubsystem.index(indexSpeed);
        } else {
            m_intakeSubsystem.stopMotors();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.setSetpoint(0);
        m_intakeSubsystem.stopMotors();
    }

}
