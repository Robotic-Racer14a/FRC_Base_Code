package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;

public class IntermediatePoseObject {

    private final Pose2d pose;
    private final double radius;
    
    public IntermediatePoseObject(Pose2d pose, double radius) {
        this.pose = pose;
        this.radius = radius;
    }

    public IntermediatePoseObject(Pose2d pose) {
        this(pose, 0);
    }

    public Pose2d getIntermediatePose() {
        return pose;
    }

    public double getRadius() {
        return radius;
    }
}
