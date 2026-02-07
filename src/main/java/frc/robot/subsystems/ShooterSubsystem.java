// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ShooterSubsystem extends SubsystemBase {
    private TalonFX shooterMotor = new TalonFX(7);
    // private PIDController closedLoopController = new PIDController(.035, 0, 0);
    private double goalRPM = 0;
    private Slot0Configs slot0Configs = new Slot0Configs();
    private double kP = 0.1;

    public static final double MAX_RPM = 6000;
    public static final double MIN_RPM = 0;

    /** Creates a new Shooter. */
    public ShooterSubsystem() {
        // closedLoopController.setTolerance(1);
        slot0Configs.kP = 2.2;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0.002;

        shooterMotor.getConfigurator().apply(slot0Configs);
    }

    @Override
    public void periodic() {
        updateDashboard();
    }

    public double getCurrentRPM() {
        return shooterMotor.getVelocity().getValue().in(RPM);
    }

    public void setSetpoint(double goal) {
        double clampedGoal = clampRPM(goal);
        double goalInRPS = clampedGoal / 60;
        VelocityVoltage request = new VelocityVoltage(0).withSlot(0);
        shooterMotor.setControl(request.withVelocity(goalInRPS));
        this.goalRPM = clampedGoal;
    }

    public void stopControl() {
        shooterMotor.setControl(new NeutralOut());
    }

    private double clampRPM(double goal) {
        return Math.max(MIN_RPM, Math.min(MAX_RPM, goal));
    }

    private void updateDashboard() {
        SmartDashboard.putNumber("Shooter Motor RPM", getCurrentRPM());
        SmartDashboard.putNumber("Current Shooter Setpoint", goalRPM);
        SmartDashboard.putNumber("Shooter kP", kP);
        // kP = SmartDashboard.getNumber("Shooter kP", 0.1);
        // System.out.println("Current Shooter kP: " + kP);
    }
}
