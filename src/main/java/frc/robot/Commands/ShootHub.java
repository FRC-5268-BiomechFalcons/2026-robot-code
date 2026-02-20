package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class ShootHub extends Command {
    ShooterSubsystem shooter;
    double goalRpm;
    IntakeSubsystem intakeSubsystem;
    double indexSpeed;
    PIDController rotController;
    DriveSubsystem driveSubsystem;
    double hubVector;

    public ShootHub(ShooterSubsystem shooter, IntakeSubsystem intakeSubsystem, DriveSubsystem driveSubsystem,
            double indexSpeed) {
        this.shooter = shooter;
        this.intakeSubsystem = intakeSubsystem;
        this.indexSpeed = indexSpeed;
        this.driveSubsystem = driveSubsystem;

        addRequirements(shooter, driveSubsystem, intakeSubsystem);
        hubVector = driveSubsystem.getHubVectorAngle();
        goalRpm = (60 * driveSubsystem.findProjectileTrajectoryVelocity()) /
            (2 * Math.PI * Constants.RobotConstants.kShooterRadius);
        rotController = new PIDController(0.014, 0, 0.0001);
        rotController.setTolerance(1.5);
    }

    @Override
    public void initialize() {
        shooter.setSetpoint(goalRpm);
        rotController.setSetpoint(hubVector);
    }

    @Override
    public void execute() {
        double rot = rotController.calculate(driveSubsystem.getHeading());
        driveSubsystem.drive(0, 0, rot, false);

        if (shooter.hitRPMSetpoint(goalRpm) && rotController.atSetpoint()) {
            intakeSubsystem.index(indexSpeed);
        } else {
            intakeSubsystem.stopMotors();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopControl();
        intakeSubsystem.stopMotors();
    }

}
