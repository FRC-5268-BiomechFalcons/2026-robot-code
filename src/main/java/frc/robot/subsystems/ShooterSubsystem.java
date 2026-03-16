// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;


public class ShooterSubsystem extends SubsystemBase {
    // Minimum and Maximum RPM capability. This is what the RPM is clamped to.
    public static final double MAX_RPM = 6000;
    public static final double MIN_RPM = 0;

    // Shooter motor.
    private final TalonFX shooterMotor = new TalonFX(7);

    // Default RPM
    private double rpm = 3000;

    // Configuration for the shooter motor.
    private Slot0Configs slot0Configs = new Slot0Configs();

    /** Creates a new Shooter. */
    public ShooterSubsystem() {
        // Sets the FeedForward and PID for the shooter motor so that the RPM setpoint hits.
        slot0Configs.kS = RobotConstants.shooterkS;
        slot0Configs.kV = RobotConstants.shooterkV;
        slot0Configs.kP = RobotConstants.shooterkP;
        slot0Configs.kI = RobotConstants.shooterkI;
        slot0Configs.kD = RobotConstants.shooterkD;

        shooterMotor.getConfigurator().apply(slot0Configs);
    }

    @Override
    public void periodic() {
        updateDashboard();
    }

    /**
     * Fetches the current RPM of the shooter motor
     * 
     * @return double - The current shooter RPM 
     */
    public double getCurrentRPM() {
        return shooterMotor.getVelocity().getValue().in(RPM);
    }

    /**
     * Fetches the current RPM setpoint.
     * 
     * @return double - Current RPM setpoint.
     */
    public double getUpdatingRPM() {
        return rpm;
    }

    /**
     * Updates the RPM setpoint
     * 
     * @param newRPM - New RPM setpoint
     */
    public void updateRPM(double newRPM) {
        rpm = newRPM;
    }

    /**
     * Runs the shooter to the current RPM setpoint. 
     */
    public void shoot() {
        double clampedGoal = MathUtil.clamp(rpm, MIN_RPM, MAX_RPM);
        double goalInRPS = clampedGoal / 60;
        VelocityVoltage request = new VelocityVoltage(0).withSlot(0);
        shooterMotor.setControl(request.withVelocity(goalInRPS));
    }

    /**
     * Stops running the shooter.
     */
    public void stopControl() {
        shooterMotor.setControl(new NeutralOut());
    }

    /**
     * Checks whether or not the shooter hit the goal RPM setpoint
     * 
     * @return boolean - Whether or not the RPM setpoint was reached
     */
    public boolean hitRPMSetpoint() {
        return getCurrentRPM() >= rpm;
    }

    /**
     * Updates the dashboard for debugging purposes
     */
    private void updateDashboard() {
        SmartDashboard.putNumber("Shooter Motor RPM", getCurrentRPM());
        SmartDashboard.putNumber("Current RPM Setpoint", rpm);
    }
}