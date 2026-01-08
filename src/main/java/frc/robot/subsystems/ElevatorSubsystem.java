package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SystemVariables;
import frc.robot.SystemVariables.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

    private TalonFX elevatorMotor = new TalonFX(27, "Upper");
    private TalonFX elevatorFollower = new TalonFX(28, "Upper");
    private PIDController elevatorController = new PIDController(ElevatorConstants.KP, ElevatorConstants.KI, ElevatorConstants.KD);
    private AnalogInput stringPot = new AnalogInput(3);
    private double targetPose = 0;
    private final GenericEntry kP, kI, kD;
    
    public ElevatorSubsystem () {

        kP =
            Shuffleboard.getTab("ElevatorPID")
                .add("kP", ElevatorConstants.KP)
                .withWidget("Number Slider")
                .withPosition(1, 1)
                .withSize(2, 1)
                .getEntry();

        kI =
            Shuffleboard.getTab("ElevatorPID")
                .add("kI", ElevatorConstants.KI)
                .withWidget("Number Slider")
                .withPosition(1, 2)
                .withSize(2, 1)
                .getEntry();

        kD =
            Shuffleboard.getTab("ElevatorPID")
                .add("kD", ElevatorConstants.KD)
                .withWidget("Number Slider")
                .withPosition(1, 3)
                .withSize(2, 1)
                .getEntry();

        var currentConfigs = new MotorOutputConfigs();

        currentConfigs.Inverted = InvertedValue.Clockwise_Positive;
        currentConfigs.NeutralMode = NeutralModeValue.Brake;
        
        elevatorMotor.getConfigurator().apply(currentConfigs);
        elevatorFollower.getConfigurator().apply(currentConfigs);
        elevatorFollower.setControl(new Follower(27, true));

        TalonFX armMotor = new TalonFX(40, "Upper");
        armMotor.getConfigurator().apply(currentConfigs);
        armMotor.close();
    }

    @Override
    public void periodic() {
        updatePIDFromDash();
        updateSmartDashboard();
        SystemVariables.elevatorAtTarget = isElevatorAtTarget();
    }


    ///////////////////// Getters /////////////////////////////////
    
    public double getTargetPose() {
        return targetPose;
    }

    public double getCurrentPoseTicks() {
        return stringPot.getValue();
    }

    public double getCurrentPoseInches() {
        return (getCurrentPoseTicks() / ElevatorConstants.TICKS_PER_INCH) - ElevatorConstants.STRINGPOT_ZERO;
    }

    public boolean isElevatorAtTarget() {
        return Math.abs(getTargetPose() - getCurrentPoseInches()) < ElevatorConstants.POSE_TOLERANCE;
    }

    ///////////////////// Setters ////////////////////////////////

    public void setTargetPose(double targetPose) {
        this.targetPose = targetPose;
    }

    public void setElevatorPower(double power) {
        power += ElevatorConstants.STATIC_FEEDFORWARD;
        power += ElevatorConstants.VELO_FEEDFORWARD * elevatorMotor.getVelocity().getValueAsDouble();
        power = Math.copySign(
            Math.min(Math.abs(power), ElevatorConstants.MAX_POWER), 
            power
        );
        //System.out.println(power);
        elevatorMotor.set(power);
    }

    ///////////////////// Logic Methods //////////////////////////

    public void runWithPID() {
        setElevatorPower(
            elevatorController.calculate(getCurrentPoseInches(), getTargetPose())
        );
    }

    ///////////////////// Internal Methods ///////////////////////

    private void updatePIDFromDash() {
        elevatorController.setPID(
            kP.getDouble(ElevatorConstants.KP),
            kI.getDouble(ElevatorConstants.KI),
            kD.getDouble(ElevatorConstants.KD)
        );
    }

    private void updateSmartDashboard() {
        SmartDashboard.putNumber("Elevator/Elevator Target", getTargetPose());
        SmartDashboard.putNumber("Elevator/Elevator Current Inches", getCurrentPoseInches());
        SmartDashboard.putNumber("Elevator/Elevator Current Ticks", getCurrentPoseTicks());
    }

}