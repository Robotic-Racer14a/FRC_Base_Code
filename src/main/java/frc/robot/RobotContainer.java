// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.drive.DriveToPose;
import frc.robot.commands.drive.FieldCentricControl;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);
    
    public final DriveSubsystem drive = TunerConstants.createDrivetrain();
    /* Path follower */
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    DriveToPoseObject[] blueOutpostOut = {
        new DriveToPoseObject(new Pose2d(4,2.5,Rotation2d.kZero), 0.25),
        new DriveToPoseObject(new Pose2d(5.5,2.5,Rotation2d.kZero), 0.25, MetersPerSecond.of(1))
    };

    DriveToPoseObject[] blueOutpostIn = {
        new DriveToPoseObject(new Pose2d(5.5,2.5,Rotation2d.k180deg), 0.5),
        new DriveToPoseObject(new Pose2d(4,2.5,Rotation2d.k180deg), 0.25, MetersPerSecond.of(1))
    };

    public RobotContainer() {
        autoChooser.setDefaultOption("Do Nothing", new WaitCommand(1));

        autoChooser.addOption("Full Collection", 
            new SequentialCommandGroup(
                new DriveToPose(drive, 
                    blueOutpostOut[0],
                    blueOutpostOut[1],
                    new DriveToPoseObject(new Pose2d(7,1,Rotation2d.kCCW_90deg), 0.5),
                    new DriveToPoseObject(new Pose2d(8,4,Rotation2d.kCCW_90deg), 0.5),
                    new DriveToPoseObject(new Pose2d(6.5,4,Rotation2d.k180deg), 0.5),
                    new DriveToPoseObject(new Pose2d(6,3,Rotation2d.kCW_90deg), 0.5),
                    blueOutpostIn[0],
                    blueOutpostIn[1],
                    new DriveToPoseObject(new Pose2d(1,1,Rotation2d.kZero))
        )));

        autoChooser.addOption("Partial Collection", 
            new SequentialCommandGroup(
                new DriveToPose(drive, 
                    blueOutpostOut[0],
                    blueOutpostOut[1],
                    new DriveToPoseObject(new Pose2d(6.5,4,Rotation2d.kZero), 0.5),
                    new DriveToPoseObject(new Pose2d(7.5,3.25,Rotation2d.kCW_90deg), 0.5),
                    new DriveToPoseObject(new Pose2d(6.5,2.5,Rotation2d.k180deg), 0.5),
                    blueOutpostIn[0],
                    blueOutpostIn[1],
                    new DriveToPoseObject(new Pose2d(1,1,Rotation2d.kZero))
        )));

        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();    
    }

    private void configureBindings() {

        drive.setDefaultCommand(new FieldCentricControl(drive, driverController));
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
