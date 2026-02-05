package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;

import com.revrobotics.spark.SparkMax;
import frc.robot.subsystems.IntakeSubsystem;


public class Intake extends Command {
    IntakeSubsystem m_intakeSubsystem;
    double speed;

    public Intake(IntakeSubsystem m_IntakeSubsystem, double speed) {
        this.m_intakeSubsystem = m_IntakeSubsystem;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        m_intakeSubsystem.intake(speed);
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeSubsystem.stopMotors();
    }

}
