package frc.robot.commands.drive;

import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DriveToPoseObject;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToPose extends Command {

    private final DriveSubsystem drive;
    private final DriveToPoseObject[] targetPoses;

    private final PIDController translationalController = new PIDController(0.001, 0, 0);
    private final PhoenixPIDController rotationalController = new PhoenixPIDController(0.001, 0, 0);
    
    public DriveToPose (DriveSubsystem drive, DriveToPoseObject... intermediatePoses) {
        this.drive = drive;
        this.targetPoses = intermediatePoses;  
        addRequirements(drive);
    }


    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
