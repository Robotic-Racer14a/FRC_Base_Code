package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;

public class DriveToPoseObject {

    private static final LinearVelocity DEFAULT_MAX_SPEED = MetersPerSecond.of(3);
    private static final AngularVelocity DEFAULT_MAX_ANGULAR_SPEED = RevolutionsPerSecond.of(1);

    private final Pose2d pose;
    private final double distanceUntilBypass;
    private final LinearVelocity stepSpeed;
    private final AngularVelocity stepAngleSpeed;
    
    public DriveToPoseObject(Pose2d pose, double distanceUntilBypass, LinearVelocity stepSpeed, AngularVelocity stepAngleSpeed) {
        this.pose = pose;
        this.distanceUntilBypass = distanceUntilBypass;
        this.stepSpeed = stepSpeed;
        this.stepAngleSpeed = stepAngleSpeed;
    }

    public DriveToPoseObject(Pose2d pose, double distanceUntilBypass, LinearVelocity stepSpeed) {
        this(pose, distanceUntilBypass, stepSpeed, DEFAULT_MAX_ANGULAR_SPEED);
    }

    public DriveToPoseObject(Pose2d pose, double distanceUntilBypass, AngularVelocity stepAngleSpeed) {
        this(pose, distanceUntilBypass, DEFAULT_MAX_SPEED, stepAngleSpeed);
    }

    public DriveToPoseObject(Pose2d pose, double distanceUntilBypass) {
        this(pose, distanceUntilBypass, DEFAULT_MAX_SPEED, DEFAULT_MAX_ANGULAR_SPEED);
    }

    public DriveToPoseObject(Pose2d pose, LinearVelocity stepSpeed) {
        this(pose, 0, stepSpeed, DEFAULT_MAX_ANGULAR_SPEED);
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

    public boolean isFineMove() {
        return getDistanceUntilBypass() == 0;
    }

    public LinearVelocity getMaxSpeed() {
        return stepSpeed;
    }

    public AngularVelocity getAngularSpeed() {
        return stepAngleSpeed;
    }
}
