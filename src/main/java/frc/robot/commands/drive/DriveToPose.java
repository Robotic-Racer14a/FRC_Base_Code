package frc.robot.commands.drive;

import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToPose extends Command {

    private final DriveSubsystem drive;
    private final Pose2d targetPose;

    private final PIDController translationalController = new PIDController(0.001, 0, 0);
    private final PhoenixPIDController rotationalController = new PhoenixPIDController(0.001, 0, 0);
    
    public DriveToPose (DriveSubsystem drive, Pose2d targetPose) {
        this.drive = drive;
        this.targetPose = targetPose;
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

        double distanceAwayX = targetPose.getX() - currentPose.getX();
        double distanceAwayY = targetPose.getY() - currentPose.getY();
        double distanceAway = Math.sqrt(Math.pow(distanceAwayX, 2) + Math.pow(distanceAwayY, 2));
        double angleOfDistance = Math.atan2(distanceAwayY, distanceAwayX);

        double translationOutput = Math.min(translationalController.calculate(distanceAway, 0), 1);

        xPow = translationOutput * Math.cos(angleOfDistance);
        yPow = translationOutput * Math.sin(angleOfDistance);

        drive.setControl(
            drive.facingAngle.withVelocityX(xPow * drive.MaxSpeed)
                             .withVelocityY(yPow * drive.MaxSpeed)
                             .withTargetDirection(targetPose.getRotation())
            );
    }

    @Override
    public void end(boolean interrupted) {
        drive.setControl(drive.brake);
    }

    @Override
    public boolean isFinished() {
        return translationalController.atSetpoint() && rotationalController.atSetpoint();
    }
}
