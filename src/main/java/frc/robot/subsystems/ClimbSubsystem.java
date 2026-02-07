// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ClimbSubsystem extends SubsystemBase {
    private SparkMax rightClimberMotor = new SparkMax(99, MotorType.kBrushless);
    private SparkMax leftClimberMotor = new SparkMax(100, MotorType.kBrushless);

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
        leftClimberMotor.set(speed);
    }

    public void stopClimber() {
        rightClimberMotor.set(0);
        leftClimberMotor.set(0);
    }
}
