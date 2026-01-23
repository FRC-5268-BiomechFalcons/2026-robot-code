package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

import com.revrobotics.spark.SparkMax;


public class Shoot extends Command {
    DriveSubsystem m_driveSubsystem;
    SparkMax m_shooterMotor;
    double power;

    public Shoot(double power, SparkMax m_shooterMotor) {
        this.power = power;
        this.m_shooterMotor = m_shooterMotor;
    }

    @Override
    public void initialize() {
        m_shooterMotor.set(power);
    }

    @Override
    public void end(boolean interrupted) {
        m_shooterMotor.set(0);
    }

}
