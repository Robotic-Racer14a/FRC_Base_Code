package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.LimelightHelpers;
import frc.robot.SystemVariables.TurretConstants;
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
            .withDriveRequestType(DriveRequestType.Velocity)
            .withHeadingPID(0.001, 0, 0); 

    public final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    ///////////////////////////////////// Drive to Pose Controllers ////////////////////////////////////
    private final PIDController translationalController = new PIDController(1, 0, 0);
    private final SlewRateLimiter accelerationLimiter = new SlewRateLimiter(2, 100, 0); // 2 Meters per second per second
   
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

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

        translationalController.setTolerance(Units.inchesToMeters(0.5));

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    @Override
    public void periodic() {

        updatePoseWithMT1();
        
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

    public Pose2d getTurretPose() {
        return this.getState().Pose.transformBy(new Transform2d(TurretConstants.offsetFromCenter, Rotation2d.kZero));
    }

    ////////////////////////////////////////////////// Drive To Pose Methods //////////////////////////////////////////////////

    /**
     * 
     * @param drivingPose Pose that will be the target pose for the translational controller
     * @param anglePose Pose that will set the angle the robot will drive in
     */
    
    public void driveToPosition(Pose2d drivingPose, Pose2d anglePose, LinearVelocity maxSpeed) {

        //Determine the sent velocity of the robot in meters per second
        double distance = distanceFromPose(drivingPose, getCurrentPose()) + distanceFromPose(drivingPose, anglePose);
        double translationalOutput = -translationalController.calculate(distance);
        translationalOutput = MathUtil.clamp(translationalOutput, -maxSpeed.in(MetersPerSecond), maxSpeed.in(MetersPerSecond));
        translationalOutput = accelerationLimiter.calculate(translationalOutput);

        //Apply velocity in the direction of the anglePose
        Rotation2d angleToPose = absoluteAngleFromPose(anglePose, getCurrentPose());
        setControl(
            driveToPoseController
                .withVelocityX(translationalOutput * Math.cos(angleToPose.getRadians()))
                .withVelocityY(translationalOutput * Math.sin(angleToPose.getRadians()))
                .withTargetDirection(anglePose.getRotation())
        );
    }

    public boolean isRobotAtTarget() {
        return translationalController.atSetpoint();
    }

    ///////////////////////////////////////////////// Limelight Methods ///////////////////////////////////////////////////////////////////////

    public void updatePoseWithMT1() {
        boolean updateVision = true;
        
        LimelightHelpers.SetRobotOrientation("limelight", getCurrentPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

        Rotation2d currentOrientation = null;

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
                currentOrientation = mt1.pose.getRotation();
            }
        }

        if (currentOrientation == null) {
            currentOrientation = getCurrentPose().getRotation();
        }

        
        
        if(mt2 != null && mt2.tagCount != 0 &&Math.abs(getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) > 720) { // if our angular velocity is greater than 720 degrees per second, ignore vision updates
          setVisionMeasurementStdDevs(VecBuilder.fill(0.15,0.15,Math.toRadians(10))); 
                addVisionMeasurement(
                    new Pose2d(mt2.pose.getTranslation(), currentOrientation),
                    Utils.fpgaToCurrentTime(mt2.timestampSeconds));
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

    public Rotation2d absoluteAngleFromPose(Pose2d measurementPose, Pose2d origin){
        return Rotation2d.fromRadians(Math.atan2(measurementPose.getY() - origin.getY(), measurementPose.getX() - origin.getX()));
    }

}
