package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    /*
     * 1. Need a motor to operate a roller
     * 2. Assign a power using a method
     * 3. Put the intake fully out
     */

     private TalonFX motor = new TalonFX(20);

     public IntakeSubsystem() {

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        motor.getConfigurator().apply(config);
     }

     //////////////////// Getters ////////////////////
     
     public double currentRollerRPM() {
        return 0;
     }

     //////////////////// Setters ////////////////////
     
     public void setMotorPower(double power) {
        motor.set(power);
     }
     
}
