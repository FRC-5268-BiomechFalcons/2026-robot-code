// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Configs;
import frc.robot.Constants.ModuleConstants;


public class MAXSwerveModule {
    // private final SparkMax m_drivingSpark;
    private final TalonFX m_drivingTalon;
    private final Slot0Configs m_driveConfigs = new Slot0Configs();
    private final SparkMax m_turningSpark;

    // private final RelativeEncoder m_drivingEncoder;
    private final AbsoluteEncoder m_turningEncoder;

    // private final SparkClosedLoopController m_drivingClosedLoopController;
    private final SparkClosedLoopController m_turningClosedLoopController;

    private double m_chassisAngularOffset = 0;
    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

    /**
     * Constructs a MAXSwerveModule and configures the driving and turning motor,
     * encoder, and PID controller. This configuration is specific to the REV
     * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
     * Encoder.
     */
    public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
        // m_drivingSpark = new SparkMax(drivingCANId, MotorType.kBrushless);
        m_drivingTalon = new TalonFX(drivingCANId);

        m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);

        // m_drivingEncoder = m_drivingSpark.getEncoder();

        m_turningEncoder = m_turningSpark.getAbsoluteEncoder();

        // m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
        m_driveConfigs.kP = 1.0;
        m_driveConfigs.kI = 0.0;
        m_driveConfigs.kD = 0.0;

        m_drivingTalon.getConfigurator().apply(m_driveConfigs);
        m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

        // Apply the respective configurations to the SPARKS. Reset parameters before
        // applying the configuration to bring the SPARK to a known good state. Persist
        // the settings to the SPARK to avoid losing them on a power cycle.
        // m_drivingSpark.configure(Configs.MAXSwerveModule.drivingConfig, ResetMode.kResetSafeParameters,
        //         PersistMode.kPersistParameters);
        m_turningSpark.configure(Configs.MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        m_chassisAngularOffset = chassisAngularOffset;
        m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
        m_drivingTalon.setPosition(0);
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.

        double currentRPS = m_drivingTalon.getVelocity().getValueAsDouble();

        double gearRatio = 10.0;
        double linearVelocity = (currentRPS * ModuleConstants.kWheelCircumferenceMeters) / gearRatio;

        return new SwerveModuleState(linearVelocity,
            new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return new SwerveModulePosition(m_drivingTalon.getPosition().getValueAsDouble(),
            new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Apply chassis angular offset to the desired state.
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

        // Optimize the reference state to avoid spinning further than 90 degrees.
        correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

        // Command driving and turning SPARKS towards their respective setpoints.
        // m_drivingClosedLoopController.setControl(correctedDesiredState.speedMetersPerSecond,
        //         ControlType.kVelocity);

        VelocityVoltage request = new VelocityVoltage(0).withSlot(0);
        m_drivingTalon.setControl(request.withVelocity(correctedDesiredState.speedMetersPerSecond));

        m_turningClosedLoopController.setSetpoint(correctedDesiredState.angle.getRadians(),
                ControlType.kPosition);

        m_desiredState = desiredState;
    }

    /** Zeroes all the SwerveModule encoders. */
    public void resetEncoders() {
        m_drivingTalon.setPosition(0);
    }
}
