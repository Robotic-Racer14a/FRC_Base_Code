package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SystemVariables;
import frc.robot.SystemVariables.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    private TalonFX intakeMotor = new TalonFX(20, "Upper");
    // private TalonFX elevatorFollower = new TalonFX(21, "Upper");
    private final GenericEntry kP, kI, kD;
    
    


    public IntakeSubsystem() {



        kP =
            Shuffleboard.getTab("ElevatorPID")
                .add("kP", IntakeConstants.KP)
                .withWidget("Text Display")
                .withPosition(1, 1)
                .withSize(2, 1)
                .getEntry();

        kI =
            Shuffleboard.getTab("ElevatorPID")
                .add("kI", IntakeConstants.KI)
                .withWidget("Text Display")
                .withPosition(1, 2)
                .withSize(2, 1)
                .getEntry();

        kD =
            Shuffleboard.getTab("ElevatorPID")
                .add("kD", IntakeConstants.KD)
                .withWidget("Text Display")
                .withPosition(1, 3)
                .withSize(2, 1)
                .getEntry();

        var currentConfigs = new MotorOutputConfigs();

        currentConfigs.Inverted = InvertedValue.Clockwise_Positive;
        currentConfigs.NeutralMode = NeutralModeValue.Coast;
        
        intakeMotor.getConfigurator().apply(currentConfigs);
        // elevatorFollower.getConfigurator().apply(currentConfigs);
        // elevatorFollower.setControl(new Follower(27, true));

    }

    @Override
    public void periodic() {
        // updatePIDFromDash();
        // updateSmartDashboard();
        // SystemVariables.elevatorAtTarget = isElevatorAtTarget();
    }

    ///////////////////// Getters /////////////////////////////////



    //////////////////// Setters /////////////////////////////////
    
    public void setIntakePower(double power) {

        intakeMotor.set(power);

    }


    
}
