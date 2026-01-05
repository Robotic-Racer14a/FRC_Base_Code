package frc.robot;

public class SystemVariables {

    public static boolean elevatorAtTarget = false;
    public static boolean armClearOfObstacles = false;
    
    public static final class ElevatorConstants {
        public static final double MAX_POWER = 1;
        public static final double KP = 0.001;
        public static final double KI = 0;
        public static final double KD = 0;
        public static final double FEEDFORWARD = 0;
        public static final double STRINGPOT_ZERO = 0;
        public static final double TICKS_PER_INCH = 50;
        public static final double POSE_TOLERANCE = .25;
    }
}
