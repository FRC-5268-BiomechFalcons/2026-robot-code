package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlign extends Command {
    Pose3d targetPose;
    DriveSubsystem driveSubsystem;
    PIDController xController;
    Limelight limelight;
    boolean isLeft;
    int mode;
    long startTime;

    public AutoAlign(Limelight limelight, DriveSubsystem driveSubsystem, boolean isLeft) {
        this.driveSubsystem = driveSubsystem;
        this.limelight = limelight;
        this.isLeft = isLeft;
        addRequirements(driveSubsystem, limelight);
        if (isLeft) {
            xController = new PIDController(0.014, 0, 0.001);
        } else {
            xController = new PIDController(0.018, 0, 0.001);
        }
        xController.setTolerance(1.5);
    }

    @Override
    public void initialize() {
        targetPose = new Pose3d();
        startTime = System.currentTimeMillis();
        System.out.println(targetPose);
        System.out.println("Running AutoAlign command...");
        System.out.println("target x " + targetPose.getX());
        System.out.println("target y " + targetPose.getY());

    }

    @Override
    public void execute() {

    }

    // && thetaController.atSetpoint()
    //  && yController.atGoal() 
    @Override
    public boolean isFinished() {
        if (xController.atSetpoint() || limelight.getAprilTag() == -1 ||
            Math.round((System.currentTimeMillis() - startTime) / 1000) > 5) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0, 0, 0, false);
        System.out.println("AutoAlign command finished");
        System.out.println("current x " + driveSubsystem.getPose().getX());
        System.out.println("current y " + driveSubsystem.getPose().getY());
    }

}
