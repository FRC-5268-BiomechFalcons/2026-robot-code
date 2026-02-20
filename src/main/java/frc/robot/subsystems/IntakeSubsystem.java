// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeSubsystem extends SubsystemBase {

    private final SparkMax intakeMotor = new SparkMax(8, MotorType.kBrushless);
    private final SparkMax indexerMotor = new SparkMax(9, MotorType.kBrushless);
    private boolean isIntaking = false;

    /** Creates a new Intake. */
    public IntakeSubsystem() {
    }

    @Override
    public void periodic() {
        updateDashboard(isIntaking);
    }

    public void intake(double speed) {
        intakeMotor.set(speed);
        indexerMotor.set(speed);
        isIntaking = true;
    }

    public void index(double speed) {
        intakeMotor.set(speed);
        indexerMotor.set(-speed);
    }

    public void stopMotors() {
        indexerMotor.set(0);
        intakeMotor.set(0);
        isIntaking = false;
    }

    private void updateDashboard(boolean isIntaking) {
        SmartDashboard.putBoolean("Intaking?", isIntaking);
        SmartDashboard.putNumber("Indexer Current Output", indexerMotor.getOutputCurrent());
    }
}
