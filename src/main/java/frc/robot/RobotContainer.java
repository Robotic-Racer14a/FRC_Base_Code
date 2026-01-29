// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.drive.FieldCentricControl;
import frc.robot.commands.elevator.ElevatorJoystick;
import frc.robot.commands.elevator.ElevatorPID;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class RobotContainer {

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);
    
    public final DriveSubsystem drive = TunerConstants.createDrivetrain();
    public final ElevatorSubsystem elevator = new ElevatorSubsystem();

    private final Telemetry logger = new Telemetry(drive.MaxSpeed);

    /* Path follower */
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() {
        autoChooser.setDefaultOption("Do Nothing", new WaitCommand(1));

        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();    
    }

    private void configureBindings() {

        drive.setDefaultCommand(new FieldCentricControl(drive, driverController));

        elevator.setDefaultCommand(new ElevatorPID(elevator));

        driverController.a().onTrue(new InstantCommand(() -> drive.setUseMT1(true)));
        driverController.b().onTrue(new InstantCommand(() -> drive.setUseMT2(true)));
        driverController.x().onTrue(new InstantCommand(() -> drive.setUseMT1(false)));
        driverController.x().onTrue(new InstantCommand(() -> drive.setUseMT2(false)));

        Trigger opLeftY = new Trigger(() -> Math.abs(operatorController.getLeftY()) > 0.1);
        opLeftY.whileTrue(new ElevatorJoystick(elevator, () -> operatorController.getLeftY()));

        operatorController.a().onTrue(new InstantCommand(() -> elevator.setTargetPose(10)));
        operatorController.b().onTrue(new InstantCommand(() -> elevator.setTargetPose(20)));
        
        drive.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
