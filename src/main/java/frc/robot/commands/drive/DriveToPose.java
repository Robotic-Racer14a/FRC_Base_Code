package frc.robot.commands.drive;

import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToPose extends Command {

    private final DriveSubsystem drive;
    private final Pose2d endPose;
    private final Pose2d[] intermediatePoses;
    private  Pose2d targetPose;
    private int stepsLeft;
    private final int numberOfSteps;
    private double intermediateRadius = 0.5; // In Meters
    private double distanceUntilContinue = 0;

    private final PIDController translationalController = new PIDController(0.001, 0, 0);
    private final PhoenixPIDController rotationalController = new PhoenixPIDController(0.001, 0, 0);
    
    public DriveToPose (DriveSubsystem drive, Pose2d endPose, Pose2d... intermediatePoses) {
        this.drive = drive;
        this.endPose = endPose;
        this.intermediatePoses = intermediatePoses;
        stepsLeft = intermediatePoses.length;
        numberOfSteps = stepsLeft;
        addRequirements(drive);
    }

    public DriveToPose (DriveSubsystem drive, Pose2d endPose) {
        this.drive = drive;
        this.endPose = endPose;
        this.intermediatePoses = null;
        stepsLeft = 1;
        numberOfSteps = 1;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        translationalController.setTolerance(Units.inchesToMeters(1));
        rotationalController.setTolerance(Units.degreesToRadians(1));
        drive.facingAngle.HeadingController = rotationalController;
    }

    @Override
    public void execute() {
        double xPow = 0, yPow = 0;
        Pose2d currentPose = drive.getCurrentPose();
        
        if (stepsLeft == numberOfSteps) {
            targetPose = endPose;
        } else {
            targetPose = intermediatePoses[numberOfSteps - stepsLeft];
            Pose2d nextPose = stepsLeft == 1 ? endPose : intermediatePoses[numberOfSteps - stepsLeft - 1];
            double angleToPose = angleBetweenPoses(targetPose, currentPose);
            double angleFromPose = angleBetweenPoses(nextPose, currentPose);
            double angleBetweenPoses = angleToPose + angleFromPose;

            distanceUntilContinue = intermediateRadius / Math.tan(angleBetweenPoses / 2);
        }

        double distanceAway = intermediateRadius == 0 ? distanceBetweenPoses(currentPose, targetPose) : distanceBetweenPoses(currentPose, endPose);
        double translationOutput = Math.min(translationalController.calculate(distanceAway, 0), 1);
        
        double angleOfDistance = angleBetweenPoses(currentPose, targetPose);
        xPow = translationOutput * Math.cos(angleOfDistance);
        yPow = translationOutput * Math.sin(angleOfDistance);

        
        drive.setControl(
            drive.facingAngle.withVelocityX(xPow * drive.MaxSpeed)
            .withVelocityY(yPow * drive.MaxSpeed)
            .withTargetDirection(endPose.getRotation())
            );

        if (distanceUntilContinue < distanceBetweenPoses(currentPose, targetPose) && !(stepsLeft == numberOfSteps)) {
            stepsLeft--;
        }
    }

    @Override
    public void end(boolean interrupted) {
        drive.setControl(drive.brake);
    }

    @Override
    public boolean isFinished() {
        return translationalController.atSetpoint() && rotationalController.atSetpoint() && stepsLeft == 1;
    }

    public double distanceBetweenPoses(Pose2d pose1, Pose2d pose2) {
        return Math.sqrt(Math.pow(pose1.getX() - pose2.getX(), 2) + Math.pow(pose1.getY() - pose2.getY(), 2));
    }

    public double angleBetweenPoses (Pose2d pose1, Pose2d pose2) {
        return Math.atan2(Math.abs(pose1.getY() - pose2.getY()), Math.abs(pose1.getX() - pose2.getX()));
    }
}
