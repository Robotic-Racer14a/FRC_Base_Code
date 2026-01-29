package frc.robot.commands.drive;

import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DriveToPoseObject;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToPose extends Command {

    private final DriveSubsystem drive;
    private final DriveToPoseObject endPose;
    private final DriveToPoseObject[] targetPoses;
    private  Pose2d targetPose;
    private int stepsLeft;
    private final int numberOfSteps;
    private double distanceUntilContinue = 0;
    private double followingDistance = 0.5;

    private final PIDController translationalController = new PIDController(0.001, 0, 0);
    private final PhoenixPIDController rotationalController = new PhoenixPIDController(0.001, 0, 0);
    
    public DriveToPose (DriveSubsystem drive, DriveToPoseObject... intermediatePoses) {
        this.drive = drive;
        this.targetPoses = intermediatePoses;
        stepsLeft = intermediatePoses.length;
        numberOfSteps = stepsLeft;
        this.endPose = intermediatePoses[stepsLeft - 1];
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
        Pose2d currentPose = drive.getCurrentPose();
        
        //Determines target pose and when to move to the next pose
        if (stepsLeft == 1) {
            targetPose = endPose.getPose();
        } else {
            //This is done by figuring out the intersection of two tangent lines on a circle
            //Because we know that the Hypotenuse is the same for both triangles, and the radius is one side of the triangle, we know that both triangles are identical
            //This allows us to determine the other side, which is our distance away from intersection point to where we move on to next line
            //Ask Alex Clute if you have any questions

            targetPose = targetPoses[numberOfSteps - stepsLeft].getPose();
            Pose2d nextPose = targetPoses[numberOfSteps - stepsLeft - 1].getPose();
            double angleToPose = drive.absoluteAngleFromPose(targetPose, currentPose);
            double angleFromPose = drive.absoluteAngleFromPose(nextPose, currentPose);
            double angleBetweenPoses = angleToPose + angleFromPose;

            distanceUntilContinue = targetPoses[numberOfSteps - stepsLeft].getDistanceUntilBypass() / Math.tan(angleBetweenPoses / 2);
        }

        //Determines the speed the robot should be driving with
        //For Continuous moves, we don't want it to slow down, just affect direction
        //For fine moves, we want it to stop at our desired location
        double distanceAway = targetPoses[numberOfSteps - stepsLeft].getDistanceUntilBypass() == 0 ? drive.distanceFromPose(currentPose, targetPose) : drive.distanceFromPose(currentPose, endPose.getPose());
        double translationOutput = Math.min(translationalController.calculate(distanceAway, 0), targetPoses[numberOfSteps - stepsLeft].getMaxSpeedPercentage());

        //Determines the distance from target we should be driving towards
        followingDistance = Math.max(distanceAway - followingDistance, 0);

        //Determine targetPose with a rotation of the slope between points
        Pose2d previousPose = stepsLeft + 1 > numberOfSteps ? currentPose : targetPoses[numberOfSteps - stepsLeft - 1].getPose();
        Pose2d angleToPose = new Pose2d(targetPose.getX(), targetPose.getY(), Rotation2d.fromRadians(drive.absoluteAngleFromPose(previousPose, targetPose)));

        //Offset angleToPose to angle towards correct spot on line, and fix angle to be target angle
        angleToPose.transformBy(new Transform2d(0, followingDistance, Rotation2d.kZero));
        angleToPose = new Pose2d(angleToPose.getX(), angleToPose.getY(), targetPose.getRotation());
        
        //Sets the direction to drive based off of angle to pose 
        double angleOfDistance = drive.absoluteAngleFromPose(currentPose, angleToPose);

        //Sends power to chassis
        drive.setDriveToPosePower(translationOutput, angleOfDistance, targetPose.getRotation());

        //Moves on to the next step once step is complete
        if (distanceUntilContinue > drive.distanceFromPose(currentPose, targetPose) - translationalController.getErrorTolerance() && !(stepsLeft == numberOfSteps)) {
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
}
