package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DriveToPoseObject;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToPose extends Command {

    private final DriveSubsystem drive;
    private final DriveToPoseObject[] targetPoses;
    private final DriveToPoseObject endPose;
    private DriveToPoseObject targetPose;
    private int stepsCompleted = 0;
    
    
    public DriveToPose (DriveSubsystem drive, DriveToPoseObject... intermediatePoses) {
        this.drive = drive;
        this.targetPoses = intermediatePoses;  

        endPose = targetPoses[targetPoses.length - 1];

        addRequirements(drive);
    }

    @Override
    public void execute() {
        //Sets the target pose while checking to see if if its a valid pose
        targetPose = targetPoses[stepsCompleted < targetPoses.length ? stepsCompleted : targetPoses.length - 1];

        //Check for fine or continuous
        if(targetPose.isFineMove()) {
            //Drive to target pose and move on once reached
            drive.driveToPosition(targetPose.getPose(), targetPose.getPose(), targetPose.getMaxSpeed());
            if (drive.isRobotAtTarget()) stepsCompleted++;
        } else {
            //Angle to target pose and move once once within bypass range
            drive.driveToPosition(endPose.getPose(), targetPose.getPose(), targetPose.getMaxSpeed());
            if (drive.distanceFromPose(targetPose.getPose(), drive.getCurrentPose()) <= targetPose.getDistanceUntilBypass()) stepsCompleted++;
        }
    }

    @Override
    public boolean isFinished() {
        return stepsCompleted == targetPoses.length;
    }
}
