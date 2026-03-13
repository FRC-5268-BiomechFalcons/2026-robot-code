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
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;


public class ShooterSubsystem extends SubsystemBase {
    public static final double MAX_RPM = 6000;
    public static final double MIN_RPM = 0;

    private final TalonFX shooterMotor = new TalonFX(7);
    private double goalRPM = 0;
    private double rpm = 3000;
    private Slot0Configs slot0Configs = new Slot0Configs();
    private InterpolatingDoubleTreeMap shooterTable = new InterpolatingDoubleTreeMap();

    /** Creates a new Shooter. */
    public ShooterSubsystem() {
        slot0Configs.kS = RobotConstants.shooterkS;
        slot0Configs.kV = RobotConstants.shooterkV;
        slot0Configs.kP = RobotConstants.shooterkP;
        slot0Configs.kI = RobotConstants.shooterkI;
        slot0Configs.kD = RobotConstants.shooterkD;

        // Interpolation Table Entries
        shooterTable.put(1.2954, 2800.0);
        shooterTable.put(1.651, 3000.0);
        shooterTable.put(2.1, 3300.0);
        shooterTable.put(2.87, 3700.0);
        shooterTable.put(3.3, 3900.0);
        shooterTable.put(4.04, 4300.0);

        //        shooterTable.put(1.8796, 3000.0);
        // shooterTable.put(2.2352, 3200.0);
        // shooterTable.put(2.68224, 3500.0);
        // shooterTable.put(3.884, 3900.0);
        // shooterTable.put(3.63, 4000.0);
        // shooterTable.put(4.369, 4300.0);

        shooterMotor.getConfigurator().apply(slot0Configs);
    }

    @Override
    public void periodic() {
        updateDashboard();
    }

    public double getCurrentRPM() {
        return shooterMotor.getVelocity().getValue().in(RPM);
    }

    public double getUpdatingRPM() {
        return rpm;
    }

    public void updateRPM(double newRPM) {
        rpm = newRPM;
    }

    public void setSetpoint() {
        double clampedGoal = MathUtil.clamp(rpm, MIN_RPM, MAX_RPM);
        double goalInRPS = clampedGoal / 60;
        VelocityVoltage request = new VelocityVoltage(0).withSlot(0);
        shooterMotor.setControl(request.withVelocity(goalInRPS));
        this.goalRPM = clampedGoal;
    }

    public void stopControl() {
        shooterMotor.setControl(new NeutralOut());
    }

    public boolean hitRPMSetpoint() {
        return getCurrentRPM() >= rpm;
    }

    private void updateDashboard() {
        SmartDashboard.putNumber("Shooter Motor RPM", getCurrentRPM());
        SmartDashboard.putNumber("Current Shooter Setpoint", goalRPM);
        SmartDashboard.putString("Motor Temp", shooterMotor.getDeviceTemp().getValueAsDouble() + " °C");
        SmartDashboard.putNumber("Current RPM Setpoint", rpm);
    }

    public double getTableRPM(double distance) {
        return shooterTable.get(distance);
    }
}