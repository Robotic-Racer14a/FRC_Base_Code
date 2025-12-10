package frc.robot.commands.drive;

import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.IntermediatePoseObject;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToPose extends Command {

    private final DriveSubsystem drive;
    private final Pose2d endPose;
    private final IntermediatePoseObject[] intermediatePoses;
    private  Pose2d targetPose;
    private int stepsLeft;
    private final int numberOfSteps;
    private double distanceUntilContinue = 0;
    private double followingDistance = 0.5;

    private final PIDController translationalController = new PIDController(0.001, 0, 0);
    private final PhoenixPIDController rotationalController = new PhoenixPIDController(0.001, 0, 0);
    
    public DriveToPose (DriveSubsystem drive, Pose2d endPose, IntermediatePoseObject... intermediatePoses) {
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
        drive.driveToPoseController.HeadingController = rotationalController;
    }

    @Override
    public void execute() {
        double xPow = 0, yPow = 0;
        Pose2d currentPose = drive.getCurrentPose();
        
        //Determines target pose and when to move to the next pose
        if (stepsLeft == numberOfSteps) {
            targetPose = endPose;
        } else {
            //This is done by figuring out the intersection of two tangent lines on a circle
            //Because we know that the Hypotenuse is the same for both triangles, and the radius is one side of the triangle, we know that both triangles are identical
            //This allows us to determine the other side, which is our distance away from intersection point to where we move on to next line
            //Ask Alex Clute if you have any questions

            targetPose = intermediatePoses[numberOfSteps - stepsLeft].getIntermediatePose();
            Pose2d nextPose = stepsLeft == 1 ? endPose : intermediatePoses[numberOfSteps - stepsLeft - 1].getIntermediatePose();
            double angleToPose = angleBetweenPoses(targetPose, currentPose);
            double angleFromPose = angleBetweenPoses(nextPose, currentPose);
            double angleBetweenPoses = angleToPose + angleFromPose;

            distanceUntilContinue = intermediatePoses[numberOfSteps - stepsLeft].getRadius() / Math.tan(angleBetweenPoses / 2);
        }

        //Determines the speed the robot should be driving with
        //For Continuous moves, we don't want it to slow down, just affect direction
        //For fine moves, we want it to stop at our desired location
        double distanceAway = intermediatePoses[numberOfSteps - stepsLeft].getRadius() == 0 ? distanceBetweenPoses(currentPose, targetPose) : distanceBetweenPoses(currentPose, endPose);
        double translationOutput = Math.min(translationalController.calculate(distanceAway, 0), 1);

        //Determines the distance from target we should be driving towards
        followingDistance = Math.max(distanceAway - followingDistance, 0);

        //Determine targetPose with a rotation of the slope between points
        Pose2d previousPose = stepsLeft + 1 > numberOfSteps ? currentPose : intermediatePoses[numberOfSteps - stepsLeft - 1].getIntermediatePose();
        Pose2d angleToPose = new Pose2d(targetPose.getX(), targetPose.getY(), Rotation2d.fromRadians(angleBetweenPoses(previousPose, targetPose)));

        //Offset angleToPose to angle towards correct spot on line, and fix angle to be target angle
        angleToPose.transformBy(new Transform2d(0, followingDistance, Rotation2d.kZero));
        angleToPose = new Pose2d(angleToPose.getX(), angleToPose.getY(), targetPose.getRotation());
        
        //Sets the direction to drive based off of angle to pose 
        double angleOfDistance = angleBetweenPoses(currentPose, angleToPose);

        //Sends power to chassis
        drive.setDriveToPosePower(translationOutput, angleOfDistance, targetPose.getRotation());

        //Moves on to the next step once step is complete
        if (distanceUntilContinue > distanceBetweenPoses(currentPose, targetPose) - translationalController.getErrorTolerance() && !(stepsLeft == numberOfSteps)) {
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
