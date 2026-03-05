package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;


public class LockOn extends Command {
    PIDController rotController;
    DriveSubsystem driveSubsystem;
    double hubVector;
    CommandXboxController driverController;

    public LockOn(DriveSubsystem driveSubsystem, CommandXboxController driverController) {
        this.driveSubsystem = driveSubsystem;
        this.driverController = driverController;

        addRequirements(driveSubsystem);

        rotController = new PIDController(0.03, 0, 0.0001);
        rotController.setTolerance(1.5);
    }

    @Override
    public void execute() {
        hubVector = driveSubsystem.getHubVectorAngle();
        rotController.setSetpoint(hubVector);
        double rot = rotController.calculate(driveSubsystem.getHeading());
        driveSubsystem.drive(
                -MathUtil.applyDeadband(Math.pow(driverController.getLeftY(), 3), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(Math.pow(driverController.getLeftX(), 3), OIConstants.kDriveDeadband),
                rot, false);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0, 0, 0, false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}