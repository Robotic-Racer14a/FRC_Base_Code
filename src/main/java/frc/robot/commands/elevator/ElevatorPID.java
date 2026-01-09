package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SystemVariables;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorPID extends Command{

    private ElevatorSubsystem elevator;

    public ElevatorPID (ElevatorSubsystem elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }
    
    @Override
    public void initialize() {

    }
    
    @Override
    public void execute() {
        if (SystemVariables.armClearOfObstacles) {
            elevator.runWithPID();
        } else {
            elevator.setElevatorPower(0);
        }
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
    
    @Override
    public void end(boolean finished) {
        elevator.setElevatorPower(0);
    }
}
