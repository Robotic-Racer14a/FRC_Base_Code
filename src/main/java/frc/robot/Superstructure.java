package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.DriveSubsystem;

public class Superstructure extends SubsystemBase{
    
    private DriveSubsystem drive = TunerConstants.createDrivetrain();

    private boolean enableDriveToPose = false;

    public Superstructure () {

    }

    @Override
    public void periodic() {
        if (enableDriveToPose) drive.driveToPosition();
    }
    

    public void fieldCentric(double leftX, double leftY, double rightX) {
        drive.setControl(
            drive.fieldCentric.withVelocityX(joystickWithDeadband(leftX, 0.1) * drive.MaxSpeed)
                              .withVelocityY(joystickWithDeadband(leftY, 0.1) * drive.MaxSpeed)
                              .withRotationalRate(joystickWithDeadband(rightX, 0.1) * drive.MaxAngularRate)
            );
    }

    public void enableDriveToPose() {
        enableDriveToPose = true;
    }

    public void disableDriveToPose() {
        enableDriveToPose = false;
    }

    public void setDriveTarget(Pose2d targetPose) {
        drive.setNewTarget(targetPose);
    }



    private double joystickWithDeadband(double input, double deadband) {

      //y = mx + b
      double m = 1 / (1 - deadband);
      double b = 1 - m;
      double y = Math.copySign((Math.abs(input) * m) + b, input);
      if (Math.abs(input) < deadband) y = 0;

      return y;
  }
}
