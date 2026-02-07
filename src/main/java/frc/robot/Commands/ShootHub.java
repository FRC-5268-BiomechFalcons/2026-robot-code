package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class ShootHub extends Command {
    ShooterSubsystem m_shooter;
    double goalRpm;
    IntakeSubsystem m_intakeSubsystem;
    double indexSpeed;
    DriveSubsystem m_DriveSubsystem;
    double hubVector;

    public ShootHub(ShooterSubsystem m_shooter, IntakeSubsystem m_intakeSubsystem,
            DriveSubsystem m_DriveSubsystem, double indexSpeed) {
        this.m_shooter = m_shooter;
        this.m_intakeSubsystem = m_intakeSubsystem;
        this.indexSpeed = indexSpeed;
        this.m_DriveSubsystem = m_DriveSubsystem;

        addRequirements(m_shooter, m_DriveSubsystem, m_intakeSubsystem);
        hubVector = m_DriveSubsystem.getHubVectorAngle();
        goalRpm = (60 * m_DriveSubsystem.findProjectileTrajectoryVelocity()) /
            (2 * Math.PI * Constants.RobotConstants.kShooterRadius);
    }

    @Override
    public void initialize() {
        m_DriveSubsystem.drive(0, 0, hubVector, false);
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
