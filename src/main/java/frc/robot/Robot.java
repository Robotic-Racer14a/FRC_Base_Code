// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Robot extends TimedRobot {
  Superstructure robot = new Superstructure();
  CommandXboxController driverController = new CommandXboxController(0);

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

  int step = 1;
  Timer timer = new Timer();

  @Override
  public void autonomousInit() {
    timer.start();
    robot.enableDriveToPose();
  }

  @Override
  public void autonomousPeriodic() {
    if (step == 1){
        robot.setDriveTarget(new Pose2d(1, 1, Rotation2d.kZero));
        if (robot.isRobotAtTarget()) {
          timer.reset();
          step = 2;
        }
          
    } else if (step == 2){
        if (timer.get() > 1) {
          step = 3;
        }
    } else if (step == 3) {
      robot.setDriveTarget(new Pose2d(2, 2, Rotation2d.kZero));
        if (robot.isRobotAtTarget()) {
          timer.reset();
          step = 4;
        }
    } else if (step == 4){
        if (timer.get() > 1) {
          step = 1;
        }
    } 
  }

  @Override
  public void autonomousExit() {
    robot.disableDriveToPose();
  }

  @Override
  public void teleopInit() {
    robot.disableDriveToPose();
  }

  @Override
  public void teleopPeriodic() {
    robot.fieldCentric(driverController.getLeftX(), driverController.getLeftY(), driverController.getRightX());
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

  
  
}
