// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ClimbSubsystem extends SubsystemBase {
    private SparkMax rightClimberMotor = new SparkMax(20, MotorType.kBrushless);
    private SparkMax leftClimberMotor = new SparkMax(21, MotorType.kBrushless);
    private PWMVictorSPX leftHookMotor = new PWMVictorSPX(0);
    private PWMVictorSPX rightHookMotor = new PWMVictorSPX(1);

    /** Creates a new ClimbSubsystem. */
    public ClimbSubsystem() {
        updateDashboard();
    }

    @Override
    public void periodic() {
    }

    private void updateDashboard() {
    }

    public void runClimber(double speed) {
        rightClimberMotor.set(speed);
        leftClimberMotor.set(-speed);
    }

    public void stopClimber() {
        rightClimberMotor.set(0);
        leftClimberMotor.set(0);
    }

    public void runHook(double speed) {
        rightHookMotor.set(speed);
        leftHookMotor.set(-speed);
    }

    public void runLeftHook(double speed) {
        leftHookMotor.set(-speed);
    }

    public void runRightHook(double speed) {
        rightHookMotor.set(speed);
    }

    public void stopHook() {
        rightHookMotor.set(0);
        leftHookMotor.set(0);
    }
}