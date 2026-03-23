package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DriveToPoseObject;
import frc.robot.SystemVariables.DrivetrainConstants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Drive the robot in a straight line towards a target pose with two modes
 * <p>
 * Fine Move - Goes from pose to pose slowing to a stop at each pose
 * <p>
 * Continous Move - Drives in the direction of the next pose while the speed is driven by the end pose
 * and will move to the next pose when it reaches the desginated distance
 */
public class DriveToPose extends Command {

    private final DriveSubsystem drive;
    private final DriveToPoseObject[] targetPoses;
    private final DriveToPoseObject endPose;
    private DriveToPoseObject targetPose;
    private int stepsCompleted = 0;
    
    StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
        .getStructTopic("Drive To Pose Target", Pose2d.struct).publish();
    
        /**
         * Creates an instance of the Drive To Pose Command
         * @param drive Current instance of the drive subsystem
         * @param intermediatePoses Drive To Pose targets that defines where and how the robot drives
         */
    public DriveToPose (DriveSubsystem drive, DriveToPoseObject... intermediatePoses) {
        this.drive = drive;
        this.targetPoses = intermediatePoses;  

        endPose = targetPoses[targetPoses.length - 1];

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        stepsCompleted = 0;
    }

    @Override
    public void execute() {
        //Sets the target pose while checking to see if if its a valid pose
        targetPose = targetPoses[stepsCompleted < targetPoses.length ? stepsCompleted : targetPoses.length - 1];

        publisher.set(targetPose.getPose());

        //Check for fine or continuous
        if(targetPose.isFineMove() || stepsCompleted >= targetPoses.length - 1) {
            //Drive to target pose and move on once reached
            drive.driveToPosition(targetPose.getPose(), targetPose.getPose(), targetPose.getMaxSpeed(), targetPose.getAngularSpeed());
            if (drive.distanceFromPose(targetPose.getPose(), drive.getCurrentPose()) < DrivetrainConstants.DRIVE_TO_POSE_TOLERANCE && 
                Math.abs(drive.getCurrentPose().getRotation().minus(targetPose.getPose().getRotation()).getDegrees()) < 2) stepsCompleted++;
        } else {
            //Angle to target pose and move once once within bypass range
            drive.driveToPosition(endPose.getPose(), targetPose.getPose(), targetPose.getMaxSpeed(), targetPose.getAngularSpeed());
            if (drive.distanceFromPose(targetPose.getPose(), drive.getCurrentPose()) <= targetPose.getDistanceUntilBypass()) stepsCompleted++;
        }
    }

    @Override
    public boolean isFinished() {
        return stepsCompleted == targetPoses.length;
    }

    @Override
    public void end(boolean finished) {
        drive.setControl(drive.fieldCentric.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    }
}
