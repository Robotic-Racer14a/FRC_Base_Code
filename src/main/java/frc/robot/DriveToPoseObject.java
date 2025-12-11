package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;

public class DriveToPoseObject {

    private final Pose2d pose;
    private final double radius;
    private final double precentageOutput;
    
    public DriveToPoseObject(Pose2d pose, double radius, double precentageOutput) {
        this.pose = pose;
        this.radius = radius;
        this.precentageOutput = precentageOutput;
    }

    public DriveToPoseObject(Pose2d pose, double radius) {
        this(pose, radius, 1);
    }

    public DriveToPoseObject(Pose2d pose) {
        this(pose, 0);
    }

    public Pose2d getPose() {
        return pose;
    }

    public double getRadius() {
        return radius;
    }

    public double getPrecentageOutput() {
        return precentageOutput;
    }
}
