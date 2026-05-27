// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.drive.DriveToPose;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private int step = 0;

  public Robot() {
    
  
    m_robotContainer = new RobotContainer();
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
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.schedule();
    // }
    m_robotContainer.drive.setDefaultCommand(new DriveToPose(m_robotContainer.drive));
    step = 0;
  }

  @Override
  public void autonomousPeriodic() {
    if (step == 0) {

      m_robotContainer.drive.setNewTarget(new Pose2d(4,2.5,Rotation2d.kZero), 0.25);
      if (m_robotContainer.drive.isRobotAtTarget()) step = 10;

    } else if (step == 10) {

      m_robotContainer.drive.setNewTarget(new Pose2d(5.5,2.5,Rotation2d.kZero), 0.25, MetersPerSecond.of(1));
      if (m_robotContainer.drive.isRobotAtTarget()) step = 20;

    } else if (step == 20) {

      m_robotContainer.drive.setNewTarget(new Pose2d(7,1,Rotation2d.kCCW_90deg), 0.75);
      if (m_robotContainer.drive.isRobotAtTarget()) step = 30;

    } else if (step == 30) {

      m_robotContainer.drive.setNewTarget(new Pose2d(8,4,Rotation2d.kCCW_90deg), 0.75);
      if (m_robotContainer.drive.isRobotAtTarget()) step = 40;

    } else if (step == 40) {

      m_robotContainer.drive.setNewTarget(new Pose2d(6.5,4,Rotation2d.k180deg), 0.75);
      if (m_robotContainer.drive.isRobotAtTarget()) step = 50;

    } else if (step == 50) {

      m_robotContainer.drive.setNewTarget(new Pose2d(6,3,Rotation2d.kCW_90deg), 0.75);
      if (m_robotContainer.drive.isRobotAtTarget()) step = 60;

    } else if (step == 60) {

      m_robotContainer.drive.setNewTarget(new Pose2d(5.5,2.5,Rotation2d.k180deg), 0.5);
      if (m_robotContainer.drive.isRobotAtTarget()) step = 70;

    } else if (step == 70) {

      m_robotContainer.drive.setNewTarget(new Pose2d(4,2.5,Rotation2d.k180deg), 0.25, MetersPerSecond.of(1));
      if (m_robotContainer.drive.isRobotAtTarget()) step = 80;

    } else if (step == 80) {

      m_robotContainer.drive.setNewTarget(new Pose2d(1,1,Rotation2d.kZero));
      if (m_robotContainer.drive.isRobotAtTarget()) step = 90;

    }
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

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
