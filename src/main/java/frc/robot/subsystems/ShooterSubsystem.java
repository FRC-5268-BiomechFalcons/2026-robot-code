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
    public static final double MAX_RPM = 6000;
    public static final double MIN_RPM = 0;

    private double carnivalRPM = 3500;
    private final TalonFX shooterMotor = new TalonFX(7);
    private double goalRPM = 0;
    private Slot0Configs slot0Configs = new Slot0Configs();

    /** Creates a new Shooter. */
    public ShooterSubsystem() {
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

    public double getCurrentRPM() {
        return shooterMotor.getVelocity().getValue().in(RPM);
    }

    public void setSetpoint(double goal) {
        // double clampedGoal = clampRPM(goal);
        // double clampedGoal = MathUtil.clamp(goal, MIN_RPM, MAX_RPM);
        double clampedGoal = MathUtil.clamp(carnivalRPM, MIN_RPM, MAX_RPM);
        double goalInRPS = clampedGoal / 60;

        VelocityVoltage request = new VelocityVoltage(0).withSlot(0);
        shooterMotor.setControl(request.withVelocity(goalInRPS));
        this.goalRPM = clampedGoal;
    }

    public void updateCarnivalRPM(double newRPM) {
        this.carnivalRPM = newRPM;
    }

    public double getCarnivalRPM() {
        return carnivalRPM;
    }

    public void stopControl() {
        shooterMotor.setControl(new NeutralOut());
    }

    public boolean hitRPMSetpoint(double goal) {
        return getCurrentRPM() >= goal - 50;
    }

    // private double clampRPM(double goal) {
    //     return Math.max(MIN_RPM, Math.min(MAX_RPM, goal));
    // }

    private void updateDashboard() {
        SmartDashboard.putNumber("Shooter Motor RPM", getCurrentRPM());
        SmartDashboard.putNumber("Current Shooter Setpoint", goalRPM);
        SmartDashboard.putString("Motor Temp", shooterMotor.getDeviceTemp().getValueAsDouble() + "°C");
        SmartDashboard.putNumber("Carnival RPM", carnivalRPM);
    }
}
