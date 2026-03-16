// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ClimbSubsystem extends SubsystemBase {
    // Climber elevator motors.
    private SparkMax rightClimberMotor = new SparkMax(20, MotorType.kBrushless);
    private SparkMax leftClimberMotor = new SparkMax(21, MotorType.kBrushless);

    // Climber hook motors.
    private PWMVictorSPX leftHookMotor = new PWMVictorSPX(0);
    private PWMVictorSPX rightHookMotor = new PWMVictorSPX(1);

    /** Creates a new ClimbSubsystem. */
    public ClimbSubsystem() {
    }

    @Override
    public void periodic() {
    }

    /**
     * Runs both climber elevator motors.
     * 
     * @param speed Desired speed of each motor.
     */
    public void runClimber(double speed) {
        rightClimberMotor.set(speed);
        leftClimberMotor.set(-speed);
    }

    /**
     * Stops both climber elevator motors.
     */
    public void stopClimber() {
        rightClimberMotor.set(0);
        leftClimberMotor.set(0);
    }

    /**
     * Runs both elevator hooks at a given speed
     * 
     * @param speed Desired speed of both hooks.
     */
    public void runHook(double speed) {
        rightHookMotor.set(speed);
        leftHookMotor.set(-speed);
    }

    /**
     * Runs solely the left climber hook.
     * 
     * @param speed Desired speed of the left hook.
     */
    public void runLeftHook(double speed) {
        leftHookMotor.set(-speed);
    }

    /**
    * Runs solely the right climber hook.
    * 
    * @param speed Desired speed of the right hook.
    */
    public void runRightHook(double speed) {
        rightHookMotor.set(speed);
    }

    /**
     * Stops both hooks on the climber.
     */
    public void stopHook() {
        rightHookMotor.set(0);
        leftHookMotor.set(0);
    }
}