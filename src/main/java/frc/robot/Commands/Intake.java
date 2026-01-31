package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

import com.revrobotics.spark.SparkMax;


public class Intake extends Command {
    DriveSubsystem m_driveSubsystem;
    SparkMax m_intakeMotor;
    SparkMax m_indexerMotor;
    double power;

    public Intake(double power, SparkMax m_intakeMotor, SparkMax m_indexerMotor) {
        this.power = power;
        this.m_intakeMotor = m_intakeMotor;
        this.m_indexerMotor = m_indexerMotor;
    }

    @Override
    public void initialize() {
        m_intakeMotor.set(power);
        m_indexerMotor.set(power);
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeMotor.set(0);
        m_indexerMotor.set(0);
    }

}
