package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.ShootOnTheFlyCalculator;;


/**
 * Autonomously rotates to face the hub and shoots fuel at a calculated RPM based off where the robot is located.
 */
public class AutoShoot extends Command {
    // Subsystems
    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intakeSubsystem;
    private final DriveSubsystem driveSubsystem;
    private final ShootOnTheFlyCalculator sotf;

    // Driver controlled inputs for x and y
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;

    private final double indexSpeed;

    // Estimated latency used for shot prediction
    private final double latencySeconds;

    // PID Controlle for rotation
    private final PIDController rotController;

    // Boolean to check whether or not the goal RPM setpoint was hit
    private boolean hitRPM;

    // Timer for revamping the shooter
    private Timer timer;

    private Timer wiggleTimer;

    private final double wigglesPerSecond = 6;

    public AutoShoot(ShooterSubsystem shooter, IntakeSubsystem intakeSubsystem, DriveSubsystem driveSubsystem,
            ShootOnTheFlyCalculator sotf, DoubleSupplier xSupplier, DoubleSupplier ySupplier,
            double indexSpeed, double latencySeconds) {

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
        this.timer = new Timer();
        this.wiggleTimer = new Timer();
        // this.agitatorTimer = new Timer();
    }

    /**
     * Starting the shooter timer every time the command gets called.
     */
    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        wiggleTimer.reset();
        wiggleTimer.start();

    }

    /**
     * Makes the robot face the hub and runs the shooter at 
     * the designated RPM based off how far the robot is from the hub.
     * This happens ever loop.
     */
    @Override
    public void execute() {
        // Current robot pose and velocity
        Translation2d robotPos = driveSubsystem.getPose().getTranslation();
        Translation2d robotVelField = driveSubsystem.getFieldRelativeVelocity();

        // Position of the hub
        Translation2d hubPos = driveSubsystem.getHubPose().getTranslation().toTranslation2d();

        // Shooter command based off the parameters above.
        ShootOnTheFlyCalculator.ShooterCommand cmd = sotf.calculate(robotPos, robotVelField, hubPos,
                latencySeconds);

        Rotation2d desiredHeading = new Rotation2d();

        // The goal heading to point towards the hub
        desiredHeading = cmd.robotHeading().plus(Rotation2d.fromDegrees(180));

        // Run the shooter at the designated RPM, calculated based on how far it is from the hub.
        double goalRpm = cmd.rpm();
        shooter.updateRPM(goalRpm);
        shooter.shoot();

        // Rotate to face the hub

        // Allow the driver to keep moving in the x and y direction 
        double x = MathUtil.clamp(xSupplier.getAsDouble(), -1.0, 1.0);
        double y = MathUtil.clamp(ySupplier.getAsDouble(), -1.0, 1.0);

        // Once we revamp the shooter (1.2 seconds) and once the robot faces the hub, index the fuel.
        if ((shooter.hitRPMSetpoint() && rotController.atSetpoint() && timer.hasElapsed(1.2)) || hitRPM) {
            hitRPM = true;
            intakeSubsystem.index(indexSpeed);

            // double power = Math.sin(2 * Math.PI * wigglesPerSecond * agitatorTimer.get());

            // rotController.setSetpoint(desiredHeading.getDegrees() + offset);
            // intakeSubsystem.agitate(power);
            driveSubsystem.setX();

        } else {
            intakeSubsystem.index(-indexSpeed / 2);
            rotController.setSetpoint(desiredHeading.getDegrees());
        }

        double rot = rotController.calculate(driveSubsystem.getHeading());
        rot = MathUtil.clamp(rot, -1.0, 1.0);
        driveSubsystem.drive(x, y, rot, true);

    }

    /**
     * Stops all the motors, halts driving, and resets the timer.
     */
    @Override
    public void end(boolean interrupted) {
        shooter.stopControl();
        intakeSubsystem.stopMotors();
        driveSubsystem.drive(0, 0, 0, false);
        timer.stop();
        timer.reset();
        hitRPM = false;
    }
}