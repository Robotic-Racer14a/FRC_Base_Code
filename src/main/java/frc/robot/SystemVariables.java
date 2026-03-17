package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;

public class SystemVariables {
    
    public static final class TurretConstants {
        public static final Translation2d offsetFromCenter = new Translation2d(
            Units.inchesToMeters(0), //Right Positive
            Units.inchesToMeters(0) //Forward Positive
        );
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
