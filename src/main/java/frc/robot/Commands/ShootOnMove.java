package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.ShootOnTheFlyCalculator;;


public class ShootOnMove extends Command {
    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intakeSubsystem;
    private final DriveSubsystem driveSubsystem;
    private final ShootOnTheFlyCalculator sotf;

    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;

    private final double indexSpeed;
    private final double latencySeconds;

    private final PIDController rotController;
    private boolean hitRPM;

    public ShootOnMove(ShooterSubsystem shooter, IntakeSubsystem intakeSubsystem,
            DriveSubsystem driveSubsystem, ShootOnTheFlyCalculator sotf, DoubleSupplier xSupplier,
            DoubleSupplier ySupplier, double indexSpeed, double latencySeconds) {

        this.shooter = shooter;
        this.intakeSubsystem = intakeSubsystem;
        this.driveSubsystem = driveSubsystem;
        this.sotf = sotf;

        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.indexSpeed = indexSpeed;
        this.latencySeconds = latencySeconds;

        addRequirements(shooter, intakeSubsystem, driveSubsystem);

        rotController = new PIDController(0.0175, 0, 0.0001);
        rotController.setTolerance(1.5);
        rotController.enableContinuousInput(-180.0, 180.0);

        this.hitRPM = false;
    }

    @Override
    public void execute() {
        Translation2d robotPos = driveSubsystem.getPose().getTranslation();
        Translation2d robotVelField = driveSubsystem.getFieldRelativeVelocity();
        Translation2d hubPos = driveSubsystem.getHubPose().getTranslation().toTranslation2d();

        ShootOnTheFlyCalculator.ShooterCommand cmd = sotf.calculate(robotPos, robotVelField, hubPos,
                latencySeconds);

        Rotation2d desiredHeading = cmd.robotHeading().plus(Rotation2d.fromDegrees(180));
        double goalRpm = cmd.rpm();
        shooter.updateRPM(goalRpm);
        shooter.setSetpoint();

        rotController.setSetpoint(desiredHeading.getDegrees());
        double rot = rotController.calculate(driveSubsystem.getHeading());
        System.out.println("Current Heading: " + driveSubsystem.getHeading());
        System.out.println("Goal Heading: " + desiredHeading.getDegrees());
        rot = MathUtil.clamp(rot, -1.0, 1.0);

        double x = MathUtil.clamp(xSupplier.getAsDouble(), -1.0, 1.0);
        double y = MathUtil.clamp(ySupplier.getAsDouble(), -1.0, 1.0);

        driveSubsystem.drive(x, y, rot, true);

        if ((shooter.hitRPMSetpoint() && rotController.atSetpoint()) || hitRPM) {
            hitRPM = true;
            intakeSubsystem.index(indexSpeed);
        } else {
            intakeSubsystem.stopMotors();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopControl();
        intakeSubsystem.stopMotors();
        driveSubsystem.drive(0, 0, 0, false);
    }
}