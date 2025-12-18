package frc.robot.commands.drive;

import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DriveToPoseObject;
import frc.robot.subsystems.DriveSubsystem;

public class CircleToPose extends Command {

    private final DriveSubsystem drive;
    private final DriveToPoseObject endPose;
    private final DriveToPoseObject[] targetPoses;
    private  Pose2d targetPose, circleCenter;
    private int stepsLeft;
    private final int numberOfSteps;
    private double distanceUntilContinue = 0;
    private double followingDistance = 0.5, circleRadius;

    private final PIDController translationalController = new PIDController(0.001, 0, 0);
    private final PhoenixPIDController rotationalController = new PhoenixPIDController(0.001, 0, 0);
    
    /**
     * Follows a circular path until getting to approach
     * This class is an example on how to use the drive to pose base for game-specific challenges, this game being reefscape
     * 
     * @param drive DriveSubsystem object
     * @param circleCenter center of circle to aviod
     * @param circleRadius radius of circle to avoid
     * @param intermediatePoses DriveToPose objects 
     */
    public CircleToPose (DriveSubsystem drive, Pose2d circleCenter, double circleRadius, DriveToPoseObject... intermediatePoses) {
        this.drive = drive;
        this.circleCenter = circleCenter;
        this.circleRadius = circleRadius;
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
            double angleToPose = angleBetweenPoses(targetPose, currentPose);
            double angleFromPose = angleBetweenPoses(nextPose, currentPose);
            double angleBetweenPoses = angleToPose + angleFromPose;

            distanceUntilContinue = targetPoses[numberOfSteps - stepsLeft].getRadius() / Math.tan(angleBetweenPoses / 2);
        }

        //Determines the speed the robot should be driving with
        //For Continuous moves, we don't want it to slow down, just affect direction
        //For fine moves, we want it to stop at our desired location
        double distanceAway = targetPoses[numberOfSteps - stepsLeft].getRadius() == 0 ? drive.distanceFromPose(currentPose, targetPose) : drive.distanceFromPose(currentPose, endPose.getPose());
        double translationOutput = Math.min(translationalController.calculate(distanceAway, 0), targetPoses[numberOfSteps - stepsLeft].getPrecentageOutput());

        //Determines the distance from target we should be driving towards
        followingDistance = Math.max(distanceAway - followingDistance, 0);

        Pose2d angleToPose = new Pose2d();
        if (drive.translationFromLine(circleCenter, currentPose, targetPose).getY() < circleRadius) {
            
            double targetDriveAngle = angleBetweenPoses(currentPose, targetPose);
            double followingCircleRadius = circleRadius + Units.inchesToMeters(20);
            double currentDistance = drive.distanceFromPose(currentPose, circleCenter);
            double centerAngle = angleBetweenPoses(currentPose, circleCenter);

            if (currentDistance > followingCircleRadius + Units.inchesToMeters(20)) { //Robot Outside of following circle, needing to get back to circle
                
                double tangentAngle = Math.acos(followingCircleRadius / currentDistance);
                if (targetDriveAngle < centerAngle) tangentAngle *= -1;

                Translation2d translationTarget = new Translation2d(
                    circleCenter.getX() + (followingCircleRadius * Math.cos(tangentAngle)),
                    circleCenter.getY() + (followingCircleRadius * Math.sin(tangentAngle))
                );

                angleToPose = new Pose2d(
                    translationTarget,
                    translationTarget.getAngle()
                );

            } else { // Following circle, needs to follow the path

                double leadingAngle = Units.degreesToRadians(10);

                if (centerAngle > targetDriveAngle) leadingAngle *= -1;
                
                angleToPose = new Pose2d(
                    followingCircleRadius * Math.cos(centerAngle + leadingAngle),
                    followingCircleRadius * Math.sin(centerAngle + leadingAngle),
                    Rotation2d.kZero
                );

            }




        } else {
            //Determine targetPose with a rotation of the slope between points
            Pose2d previousPose = stepsLeft + 1 > numberOfSteps ? currentPose : targetPoses[numberOfSteps - stepsLeft - 1].getPose();
            angleToPose = new Pose2d(targetPose.getX(), targetPose.getY(), Rotation2d.fromRadians(angleBetweenPoses(previousPose, targetPose)));
            angleToPose.transformBy(new Transform2d(0, followingDistance, Rotation2d.kZero));
        }

        //Offset angleToPose to angle towards correct spot on line, and fix angle to be target angle
        angleToPose = new Pose2d(angleToPose.getX(), angleToPose.getY(), targetPose.getRotation());
        
        //Sets the direction to drive based off of angle to pose 
        double angleOfDistance = angleBetweenPoses(currentPose, angleToPose);

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


    public double angleBetweenPoses (Pose2d pose1, Pose2d pose2) {
        return Math.atan2(Math.abs(pose1.getY() - pose2.getY()), Math.abs(pose1.getX() - pose2.getX()));
    }
}
