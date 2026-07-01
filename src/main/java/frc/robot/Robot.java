// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.DriveSubsystem;

public class Robot extends TimedRobot {
  private final DriveSubsystem drive = TunerConstants.createDrivetrain();
  CommandXboxController driverController = new CommandXboxController(0);
  private int step = 0;

  public Robot() {
    
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    
    step = 0;
  }

  @Override
  public void autonomousPeriodic() {
    drive.driveToPosition();

    if (step == 0) {

      drive.setNewTarget(new Pose2d(4,2.5,Rotation2d.kZero), 0.25);
      if (drive.isRobotAtTarget()) step = 10;

    } else if (step == 10) {

      drive.setNewTarget(new Pose2d(5.5,2.5,Rotation2d.kZero), 0.25, MetersPerSecond.of(1));
      if (drive.isRobotAtTarget()) step = 20;

    } else if (step == 20) {

      drive.setNewTarget(new Pose2d(7,1,Rotation2d.kCCW_90deg), 0.75);
      if (drive.isRobotAtTarget()) step = 30;

    } else if (step == 30) {

      drive.setNewTarget(new Pose2d(8,4,Rotation2d.kCCW_90deg), 0.75);
      if (drive.isRobotAtTarget()) step = 40;

    } else if (step == 40) {

      drive.setNewTarget(new Pose2d(6.5,4,Rotation2d.k180deg), 0.75);
      if (drive.isRobotAtTarget()) step = 50;

    } else if (step == 50) {

      drive.setNewTarget(new Pose2d(6,3,Rotation2d.kCW_90deg), 0.75);
      if (drive.isRobotAtTarget()) step = 60;

    } else if (step == 60) {

      drive.setNewTarget(new Pose2d(5.5,2.5,Rotation2d.k180deg), 0.5);
      if (drive.isRobotAtTarget()) step = 70;

    } else if (step == 70) {

      drive.setNewTarget(new Pose2d(4,2.5,Rotation2d.k180deg), 0.25, MetersPerSecond.of(1));
      if (drive.isRobotAtTarget()) step = 80;

    } else if (step == 80) {

      drive.setNewTarget(new Pose2d(1,1,Rotation2d.kZero));
      if (drive.isRobotAtTarget()) step = 90;

    }
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    drive.setControl(
            drive.fieldCentric.withVelocityX(joystickWithDeadband(driverController.getLeftX(), 0.1) * drive.MaxSpeed)
                              .withVelocityY(joystickWithDeadband(driverController.getLeftY(), 0.1) * drive.MaxSpeed)
                              .withRotationalRate(joystickWithDeadband(driverController.getRightX(), 0.1) * drive.MaxAngularRate)
            );
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}

  
  private double joystickWithDeadband(double input, double deadband) {

      //y = mx + b
      double m = 1 / (1 - deadband);
      double b = 1 - m;
      double y = Math.copySign((Math.abs(input) * m) + b, input);
      if (Math.abs(input) < deadband) y = 0;

      return y;
  }
}
