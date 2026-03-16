// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeSubsystem extends SubsystemBase {

    // Intake Motor
    private final SparkMax intakeMotor = new SparkMax(9, MotorType.kBrushless);

    // Indexer Motor
    private final SparkMax indexerMotor = new SparkMax(8, MotorType.kBrushless);

    /** Creates a new Intake. */
    public IntakeSubsystem() {
    }

    @Override
    public void periodic() {
    }

    /**
     * Runs the intake and indexer motors to intake fuel.
     * 
     * @param speed Desired speed of the intake and indexer motors.
     */
    public void intake(double speed) {
        intakeMotor.set(speed);
        indexerMotor.set(speed);
    }

    /**
    * Runs the intake and indexer motors to index fuel.
    * 
    * @param speed Desired speed of the intake and indexer motors.
    */
    public void index(double speed) {
        intakeMotor.set(speed);
        indexerMotor.set(-speed);
    }

    /**
     * Stops the intake and indexer motors.
     */
    public void stopMotors() {
        indexerMotor.set(0);
        intakeMotor.set(0);
    }
}