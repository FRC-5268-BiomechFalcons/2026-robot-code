// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ShooterSubsystem extends SubsystemBase {
    private TalonFX m_shooterMotor = new TalonFX(7);
    private PIDController m_closedLoopController = new PIDController(.08, 0, 0);
    private double goalRPM = 0;
    public static final double MAX_RPM = 6000;
    public static final double MIN_RPM = 0;

    /** Creates a new Shooter. */
    public ShooterSubsystem() {
        m_closedLoopController.setTolerance(1);
    }

    @Override
    public void periodic() {
        updateDashboard();
        if (goalRPM != 0) {
            double voltage = m_closedLoopController.calculate(getCurrentRPM());
            m_shooterMotor.setVoltage(voltage);
        } else {
            m_shooterMotor.setVoltage(0);
        }
    }

    public double getCurrentRPM() {
        return m_shooterMotor.getVelocity().getValue().in(RPM);
    }

    public void setSetpoint(double goal) {
        double clampedGoal = clampRPM(goal);
        m_closedLoopController.setSetpoint(clampedGoal);
        this.goalRPM = clampedGoal;
    }

    private double clampRPM(double goal) {
        return Math.max(MIN_RPM, Math.min(MAX_RPM, goal));
    }

    public void updateDashboard() {
        SmartDashboard.putNumber("Shooter Motor RPM", getCurrentRPM());
        SmartDashboard.putNumber("Current Shooter Setpoint", goalRPM);
    }
}
