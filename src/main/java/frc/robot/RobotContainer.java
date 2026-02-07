// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.drive.DriveToPose;
import frc.robot.commands.drive.FieldCentricControl;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);
    
    public final DriveSubsystem drive = TunerConstants.createDrivetrain();

    private final Telemetry logger = new Telemetry(drive.MaxSpeed);

    /* Path follower */
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() {
        autoChooser.setDefaultOption("Do Nothing", new WaitCommand(1));

        autoChooser.addOption("Run Poses", 
            new SequentialCommandGroup(
                new DriveToPose(drive, new DriveToPoseObject(new Pose2d(1,1,Rotation2d.kZero), 0.25)),
                new DriveToPose(drive, new DriveToPoseObject(new Pose2d(2,2,Rotation2d.kZero), 0.25)),
                new DriveToPose(drive, new DriveToPoseObject(new Pose2d(1,1,Rotation2d.kZero), 0.25))
        ));

        autoChooser.addOption("Drive", new InstantCommand(() -> drive.setControl(
            drive.fieldCentric.withVelocityX(1)
                              .withVelocityY(1)
                              .withRotationalRate(1)
            )));

        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();    
    }

    private void configureBindings() {

        drive.setDefaultCommand(new FieldCentricControl(drive, driverController));


        driverController.a().onTrue(new InstantCommand(() -> drive.setUseMT1(true)));
        driverController.b().onTrue(new InstantCommand(() -> drive.setUseMT2(true)));
        driverController.x().onTrue(new InstantCommand(() -> drive.setUseMT1(false)));
        driverController.x().onTrue(new InstantCommand(() -> drive.setUseMT2(false)));
        
        drive.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
