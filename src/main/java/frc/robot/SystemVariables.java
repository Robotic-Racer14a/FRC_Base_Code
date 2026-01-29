package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import frc.robot.generated.TunerConstants;

public class SystemVariables {

    public static boolean elevatorAtTarget = false;
    public static boolean armClearOfObstacles = true;

    //public static boolean 
    
    public static final class ElevatorConstants {
        public static final double MAX_POWER = 1;
        public static final double KP = 0.12;
        public static final double KI = 0.005;
        public static final double KD = 0.002;
        public static final double STATIC_FEEDFORWARD = 0.02;
        public static final double STRINGPOT_ZERO = 4;
        public static final double TICKS_PER_INCH = 50;
        public static final double POSE_TOLERANCE = .25;
    }

    public static final class IntakeConstants {
        public static final double KP = 0.12;
        public static final double KI = 0.005;
        public static final double KD = 0.002;
    }

    public static final class DrivetrainConstants {
        
        public static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        public static final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    }
}
