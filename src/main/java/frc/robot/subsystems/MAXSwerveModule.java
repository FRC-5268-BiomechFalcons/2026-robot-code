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
        m_driveConfigs.kP = ModuleConstants.kPKrakenDrive;
        m_driveConfigs.kI = ModuleConstants.kIKrakenDrive;
        m_driveConfigs.kD = ModuleConstants.kDKrakenDrive;

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
        double motorRps = m_drivingTalon.getVelocity().getValueAsDouble();
        double wheelRps = motorRps / ModuleConstants.kKrakenDriveGearRatio;
        double mps = wheelRps * ModuleConstants.kWheelCircumferenceMeters;

        return new SwerveModuleState(mps, getTurningAngle());
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        double motorRot = m_drivingTalon.getPosition().getValueAsDouble();
        return new SwerveModulePosition(motorRotToMeters(motorRot), getTurningAngle());
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState corrected = new SwerveModuleState(desiredState.speedMetersPerSecond,
            desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset)));

        corrected.optimize(Rotation2d.fromRadians(m_turningEncoder.getPosition()));

        VelocityVoltage request = new VelocityVoltage(0).withSlot(0);
        m_drivingTalon.setControl(request.withVelocity(mpsToMotorRps(corrected.speedMetersPerSecond)));

        m_turningClosedLoopController.setSetpoint(corrected.angle.getRadians(), ControlType.kPosition);

        m_desiredState = desiredState;
    }

    /** Zeroes all the SwerveModule encoders. */
    public void resetEncoders() {
        m_drivingTalon.setPosition(0);
    }

    private double mpsToMotorRps(double mps) {
        double wheelRps = mps / ModuleConstants.kWheelCircumferenceMeters;
        return wheelRps * ModuleConstants.kKrakenDriveGearRatio;
    }

    private double motorRotToMeters(double motorRot) {
        double wheelRot = motorRot / ModuleConstants.kKrakenDriveGearRatio;
        return wheelRot * ModuleConstants.kWheelCircumferenceMeters;
    }

    private Rotation2d getTurningAngle() {
        return Rotation2d.fromRadians(m_turningEncoder.getPosition())
                .minus(Rotation2d.fromRadians(m_chassisAngularOffset));
    }
}
