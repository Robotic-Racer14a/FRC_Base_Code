package frc.robot.commands.elevator;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SystemVariables;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorJoystick extends Command{

    private ElevatorSubsystem elevator;
    private Supplier<Double> power;

    public ElevatorJoystick (ElevatorSubsystem elevator, Supplier<Double> power) {
        this.elevator = elevator;
        this.power = power;
        addRequirements(elevator);
    }
    
    @Override
    public void initialize() {

    }
    
    @Override
    public void execute() {
        elevator.setElevatorPower(power.get() * -0.1);
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
    
    @Override
    public void end(boolean finished) {
        elevator.setElevatorPower(0);
        elevator.setTargetPose(elevator.getCurrentPose());
    }
}
