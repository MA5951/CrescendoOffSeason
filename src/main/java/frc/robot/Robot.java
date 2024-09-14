// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ma5951.utils.Logger.LoggedString;
import com.ma5951.utils.Logger.MALog;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystem.Arm.Arm;
import frc.robot.Subsystem.Arm.ArmConstants;
import frc.robot.Subsystem.LED.LED;
import frc.robot.Subsystem.PoseEstimation.PoseEstimator;
import frc.robot.Subsystem.PoseEstimation.SwervePoseCalculator;



public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  

  private RobotContainer m_robotContainer;
  private LoggedString currentRobotStateLog;
  private LoggedString lastRobotStateLog;


  @Override
  public void robotInit() {
    MALog.getInstance();
    m_robotContainer = new RobotContainer();
    m_robotContainer.setIDLE();
    Arm.getInstance().setTargetState(ArmConstants.IDLE);
    PoseEstimator.getInstance();
    SwervePoseCalculator.getInstance();
    LED.getInstance();
    addPeriodic(() -> PoseEstimator.getInstance().updateOdometry() , 0.01 , 0);
    

    currentRobotStateLog = new LoggedString("/RobotControl/Current Robot State");
    lastRobotStateLog = new LoggedString("/RobotControl/Last Robot State");
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    PoseEstimator.getInstance().update();
    SwervePoseCalculator.getInstance().update();
    RobotConstants.SUPER_STRUCTURE.updateAfterDSConnect();
    RobotConstants.SUPER_STRUCTURE.update();
    currentRobotStateLog.update(RobotContainer.currentRobotState.getName());
    lastRobotStateLog.update(RobotContainer.lastRobotState.getName());
  }

  @Override
  public void disabledInit() {
    MALog.getInstance().stopLog();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
      }
    MALog.getInstance().startLog();
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
