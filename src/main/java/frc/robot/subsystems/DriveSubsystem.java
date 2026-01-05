package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class DriveSubsystem extends TunerSwerveDrivetrain implements Subsystem {
    public double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    
    
    public final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors

    public final SwerveRequest.FieldCentricFacingAngle driveToPoseController = new SwerveRequest.FieldCentricFacingAngle()
            .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.BlueAlliance)
            .withDriveRequestType(DriveRequestType.Velocity); 

    public final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
   
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private final double maxAcceleration = 2 * .02; //2 m/s with 20 ms interval
    private final double maxTranslationalAngleChange = Units.degreesToRadians(180) * .02; //180 degrees per second with 20 ms interval
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private boolean useMT1, useMT2;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public DriveSubsystem(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    @Override
    public void periodic() {

        if (useMT1) updatePoseWithMT1();
        else if (useMT2) updatePoseWithMT2();
        
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }

    /////////////////////////////////////////////////// Getters //////////////////////////////////////////////////////////////////

    public Pose2d getCurrentPose() {
        return this.getState().Pose;
    }

    ////////////////////////////////////////////////// Setters //////////////////////////////////////////////////

    public void setDriveToPosePower(double targetVelocity, double driveAngle, Rotation2d targetAngle) {
        double currentVelocity = Math.sqrt(Math.pow(this.getState().Speeds.vyMetersPerSecond, 2) + Math.pow(this.getState().Speeds.vxMetersPerSecond, 2));
        double currentTranslationAngle = Math.atan2(this.getState().Speeds.vyMetersPerSecond, this.getState().Speeds.vxMetersPerSecond);

        targetVelocity = targetVelocity < currentVelocity ? Math.max(targetVelocity, currentVelocity - maxAcceleration) : Math.min(targetVelocity, currentVelocity + maxAcceleration);
        driveAngle = driveAngle < currentTranslationAngle ? Math.max(driveAngle, currentTranslationAngle - maxTranslationalAngleChange) : Math.min(driveAngle, currentTranslationAngle + maxTranslationalAngleChange);

        double xPow = targetVelocity * Math.cos(driveAngle);
        double yPow = targetVelocity * Math.sin(driveAngle);

        this.setControl(
            driveToPoseController.withVelocityX(xPow)
            .withVelocityY(yPow)
            .withTargetDirection(targetAngle)
            );
    }

    ///////////////////////////////////////////////// Limelight Methods ///////////////////////////////////////////////////////////////////////

    public void setUseMT1(boolean useMT1) {
        if (useMT1) useMT2 = false;
        this.useMT1 = useMT1;
    }
    
    public void setUseMT2(boolean useMT2) {
        if (useMT2) useMT1 = false;
        this.useMT2 = useMT2;
    }

    public void updatePoseWithMT1() {
        boolean updateVision = true;
        
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

        if(mt1 != null) {
          
            if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
                if(mt1.rawFiducials[0].ambiguity > .7)
                {
                    updateVision = false;
                }
                if(mt1.rawFiducials[0].distToCamera > 3)
                {
                    updateVision = false;
                }
            }
            if(mt1.tagCount == 0) {
                updateVision = false;
            }
            if(updateVision) {
                setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,Math.toRadians(1)));
                addVisionMeasurement(
                    mt1.pose,
                    Utils.fpgaToCurrentTime(mt1.timestampSeconds));
            }
        }
    }

    public void updatePoseWithMT2() {
        boolean updateVision = true;
        
        LimelightHelpers.SetRobotOrientation("limelight", getCurrentPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2_blue = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        if(Math.abs(getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) > 720) { // if our angular velocity is greater than 720 degrees per second, ignore vision updates
          updateVision = false;
        }

        if (mt2_blue != null) {
            if(mt2_blue.tagCount == 0)
            {
                updateVision = false;
            }
            if(updateVision)
            {
                
                setVisionMeasurementStdDevs(VecBuilder.fill(0.15,0.15,9999999)); //x and Y were 0.7
                addVisionMeasurement(
                    mt2_blue.pose,
                    Utils.fpgaToCurrentTime(mt2_blue.timestampSeconds));
            }
        }
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }



    /////////////////////////// Pose Utility Methods (TODO: Move to new location) //////////////////////////////////////////////////

    /**
     * Method gets the distance of a specified point from another point using a third point as a rotational reference
     * @param measurementPose
     * @param lineOrigin
     * @param lineXPos
     * @return
     */
    public Translation2d translationFromLine(Pose2d measurementPose, Pose2d lineOrigin, Pose2d lineXPos) {
        double lineAngle = Math.atan2(lineXPos.getY() - lineOrigin.getY(), lineXPos.getX() - lineOrigin.getX());
        lineXPos = lineXPos.rotateAround(lineOrigin.getTranslation(), Rotation2d.fromRadians(-lineAngle));
        measurementPose = measurementPose.rotateAround(lineOrigin.getTranslation(), Rotation2d.fromRadians(-lineAngle));

        lineXPos = lineXPos.transformBy(new Transform2d(lineOrigin.getTranslation(),  Rotation2d.kZero));
        measurementPose = measurementPose.transformBy(new Transform2d(lineOrigin.getTranslation(),  Rotation2d.kZero));

        return measurementPose.getTranslation();
    }


    public double distanceFromPose(Pose2d measurementPose, Pose2d origin){
        return Math.sqrt(Math.pow(measurementPose.getX() - origin.getX(), 2) + Math.pow(measurementPose.getY() - origin.getY(), 2));
    }

    /**
     * Returns an angle 0 through pi/2 that is the angle between the two poses
     */
    public double absoluteAngleFromPose(Pose2d measurementPose, Pose2d origin){
        return Math.atan2(Math.abs(measurementPose.getY() - origin.getY()), Math.abs(measurementPose.getX() - origin.getX()));
    }

}
