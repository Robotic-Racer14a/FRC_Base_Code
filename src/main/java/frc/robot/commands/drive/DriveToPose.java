package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
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
    
        /**
         * Creates an instance of the Drive To Pose Command
         * @param drive Current instance of the drive subsystem
         */
    public DriveToPose (DriveSubsystem drive) {
        this.drive = drive;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        drive.driveToPosition();
            
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean finished) {
        drive.setControl(drive.fieldCentric.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    }
}
