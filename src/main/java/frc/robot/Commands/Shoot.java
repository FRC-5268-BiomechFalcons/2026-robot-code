package frc.robot.Commands;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;


public class Shoot extends Command {
    DriveSubsystem m_driveSubsystem;
    TalonFX m_shooterMotor;
    double power;

    public Shoot(double power, TalonFX m_shooterMotor) {
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
