package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SystemVariables;
import frc.robot.SystemVariables.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

    private TalonFX elevatorMotor = new TalonFX(20, "CANivore");
    private TalonFX elevatorFollower = new TalonFX(28, "CANivore");

    private PIDController elevatorController = new PIDController(0.001, 0, 0);

    private AnalogInput distanceSensor = new AnalogInput(3);
    private AnalogInput stringPot = new AnalogInput(3);
    private SlewRateLimiter filter = new SlewRateLimiter(4);
    private double targetPose = 0;
    private final GenericEntry kP, kI, kD;
    
    public ElevatorSubsystem () {
        kP =
            Shuffleboard.getTab("ElevatorPID")
                .add("kP", ElevatorConstants.KP)
                .withWidget("Text Display")
                .withPosition(1, 1)
                .withSize(2, 1)
                .getEntry();

        kI =
            Shuffleboard.getTab("ElevatorPID")
                .add("kI", ElevatorConstants.KI)
                .withWidget("Text Display")
                .withPosition(1, 2)
                .withSize(2, 1)
                .getEntry();

        kD =
            Shuffleboard.getTab("ElevatorPID")
                .add("kD", ElevatorConstants.KD)
                .withWidget("Text Display")
                .withPosition(1, 3)
                .withSize(2, 1)
                .getEntry();

                
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.PeakForwardDutyCycle = 1;
        config.MotorOutput.PeakReverseDutyCycle = -0.5;

        config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        config.Slot0.kP = 0.001;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;

        config.CurrentLimits.StatorCurrentLimit = 120;
        config.CurrentLimits.SupplyCurrentLimit = 50;
        config.CurrentLimits.SupplyCurrentLowerLimit = 40;
        config.CurrentLimits.SupplyCurrentLowerTime = 0.5;
        
        elevatorMotor.getConfigurator().apply(config);


        elevatorFollower.getConfigurator().apply(config);
        elevatorFollower.setControl(new Follower(20, MotorAlignmentValue.Opposed));

        elevatorMotor.set(1);
        elevatorMotor.setControl(new DutyCycleOut(1));

        
        elevatorMotor.setControl(new VelocityVoltage(10));
        elevatorMotor.setControl(new PositionDutyCycle(10));

    }

    @Override
    public void periodic() {
        updatePIDFromDash();
        updateSmartDashboard();
        SystemVariables.elevatorAtTarget = isElevatorAtTarget();
    }


    ///////////////////// Getters /////////////////////////////////
    
    public boolean doWeHaveGamePiece() {
        return distanceSensor.getValue() < 100 && distanceSensor.getValue() != 0;
    }
    
    public double getTargetPose() {
        return targetPose;
    }

    public double getCurrentPoseTicks() {
        return stringPot.getValue();
    }

    /**
     * This method gives you the current position of the elevator in inches off of the ground
     */
    public double getCurrentPosition() {
        return (stringPot.getValue() / ElevatorConstants.TICKS_PER_INCH) - ElevatorConstants.STRINGPOT_ZERO;
    }

    public boolean isElevatorAtTarget() {
        return Math.abs(getTargetPose() - getCurrentPosition()) < ElevatorConstants.POSE_TOLERANCE;
    }

    ///////////////////// Setters ////////////////////////////////

    public void setTargetPose(double targetPose) {
        this.targetPose = targetPose;
    }

    public void setElevatorPower(double power) {
        power += ElevatorConstants.STATIC_FEEDFORWARD;

        power = Math.copySign(
            Math.min(Math.abs(power), ElevatorConstants.MAX_POWER), 
            power
        );

        power = filter.calculate(power);
        elevatorMotor.set(power);
    }

    public void setElevatorPower(double power, double maxPower) {
        power += ElevatorConstants.STATIC_FEEDFORWARD;

        power = Math.copySign(
            Math.min(Math.abs(power), maxPower), 
            power
        );

        power = filter.calculate(power);
        elevatorMotor.set(power);
    }

    ///////////////////// Logic Methods //////////////////////////

    public void runWithPID() {
        setElevatorPower(
            elevatorController.calculate(getCurrentPosition(), getTargetPose())
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


        
        SmartDashboard.putNumber("Elevator/Elevator Current Inches", getCurrentPosition());
        SmartDashboard.putNumber("Elevator/Elevator Current Ticks", getCurrentPoseTicks());
        SmartDashboard.putNumber("Elevator/Elevator Current Velo", elevatorMotor.getVelocity().getValueAsDouble());
    }

}