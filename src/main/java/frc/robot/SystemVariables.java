package frc.robot;

public class SystemVariables {

    public static boolean elevatorAtTarget = false;
    public static boolean armClearOfObstacles = true;
    
    public static final class ElevatorConstants {
        public static final double MAX_POWER = 0.3;
        public static final double KP = 0.35;
        public static final double KI = 0;
        public static final double KD = 0;
        public static final double STATIC_FEEDFORWARD = 0.02;
        public static final double VELO_FEEDFORWARD = 0;
        public static final double STRINGPOT_ZERO = 4;
        public static final double TICKS_PER_INCH = 50;
        public static final double POSE_TOLERANCE = .25;
    }
}
