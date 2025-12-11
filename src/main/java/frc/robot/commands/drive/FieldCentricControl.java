package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveSubsystem;

public class FieldCentricControl extends Command {

    private final DriveSubsystem drive;
    private final CommandXboxController driverController;
    
    public FieldCentricControl (DriveSubsystem drive, CommandXboxController driverController) {
        this.drive = drive;
        this.driverController = driverController;
        addRequirements(drive);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        drive.setControl(
            drive.fieldCentric.withVelocityX(joystickWithDeadband(driverController.getLeftX(), 0.1) * drive.MaxSpeed)
                              .withVelocityY(joystickWithDeadband(driverController.getLeftY(), 0.1) * drive.MaxSpeed)
                              .withRotationalRate(joystickWithDeadband(driverController.getRightX(), 0.1) * drive.MaxAngularRate)
            );
    }

    @Override
    public void end(boolean interrupted) {
        drive.setControl(drive.brake);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private double joystickWithDeadband(double input, double deadband) {

        //y = mx + b
        double m = 1 / (1 - deadband);
        double b = 1 - m;
        double y = Math.copySign((Math.abs(input) * m) + b, input);
        if (y < deadband) y = 0;

        return y;
    }
}
