package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.SystemVariables.DrivetrainConstants;

public class DriveToPoseObject {

    private final Pose2d pose;
    private final double distanceUntilBypass;
    private final LinearVelocity stepSpeed;
    
    public DriveToPoseObject(Pose2d pose, double distanceUntilBypass, LinearVelocity stepSpeed) {
        this.pose = pose;
        this.distanceUntilBypass = distanceUntilBypass;
        this.stepSpeed = stepSpeed;
    }

    public DriveToPoseObject(Pose2d pose, double distanceUntilBypass) {
        this(pose, distanceUntilBypass, MetersPerSecond.of(DrivetrainConstants.MaxSpeed));
    }

    public DriveToPoseObject(Pose2d pose, LinearVelocity stepSpeed) {
        this(pose, 0, stepSpeed);
    }

    public DriveToPoseObject(Pose2d pose) {
        this(pose, 0);
    }

    public Pose2d getPose() {
        return pose;
    }

    public double getDistanceUntilBypass() {
        return distanceUntilBypass;
    }

    public double getMaxSpeedPercentage() {
        return stepSpeed.in(MetersPerSecond) / DrivetrainConstants.MaxSpeed;
    }
}
