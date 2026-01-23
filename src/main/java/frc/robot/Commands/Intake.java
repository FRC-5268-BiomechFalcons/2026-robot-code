package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

import com.revrobotics.spark.SparkMax;


public class Intake extends Command {
    DriveSubsystem m_driveSubsystem;
    SparkMax m_intakeMotor;
    double power;

    public Intake(double power, SparkMax m_intakeMotor) {
        this.power = power;
        this.m_intakeMotor = m_intakeMotor;
    }

    @Override
    public void initialize() {
        m_intakeMotor.set(power);
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeMotor.set(0);
    }

}
