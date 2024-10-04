// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ma5951.utils.Logger.LoggedInt;
import com.ma5951.utils.Logger.LoggedString;
import com.ma5951.utils.Logger.MALog;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystem.Arm.Arm;
import frc.robot.Subsystem.Arm.ArmConstants;
import frc.robot.Subsystem.LED.LED;
import frc.robot.Subsystem.PoseEstimation.PoseEstimator;
import frc.robot.Subsystem.Swerve.SwerveConstants;



public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  

  private RobotContainer m_robotContainer;
  private LoggedString currentRobotStateLog;
  private LoggedString lastRobotStateLog;
  private LoggedInt currentRobotStateNumberLog;

  @Override
  public void robotInit() {
    MALog.getInstance();
    m_robotContainer = new RobotContainer();
    m_robotContainer.setIDLE();
    Arm.getInstance().setTargetState(ArmConstants.IDLE);
    PoseEstimator.getInstance();
    LED.getInstance();
    addPeriodic(() -> PoseEstimator.getInstance().updateOdometry() , 1 / SwerveConstants.ODOMETRY_UPDATE_RATE , 0);
    

    currentRobotStateLog = new LoggedString("/RobotControl/Current Robot State");
    lastRobotStateLog = new LoggedString("/RobotControl/Last Robot State");
    currentRobotStateNumberLog = new LoggedInt("/RobotControl/Current Robot State Num");
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    PoseEstimator.getInstance().update();
    RobotConstants.SUPER_STRUCTURE.update();
    currentRobotStateLog.update(RobotContainer.currentRobotState.getName());
    lastRobotStateLog.update(RobotContainer.lastRobotState.getName());
    currentRobotStateNumberLog.update(getStateAsNum());
    LED.getInstance().periodic();

    
  }

  @Override
  public void disabledInit() {
    MALog.getInstance().stopLog();
    RobotContainer.disableDeafultCommands();
  }

  @Override
  public void disabledPeriodic() {
    
  }

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

    RobotContainer.setDeafultCommands();
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

  public int getStateAsNum() {
    if (RobotContainer.currentRobotState == RobotConstants.IDLE) {
      return 0;
    } else if (RobotContainer.currentRobotState == RobotConstants.INTAKE) {
      return 2;
    } else if (RobotContainer.currentRobotState == RobotConstants.EJECT) {
      return 4;
    } else if (RobotContainer.currentRobotState == RobotConstants.WARMING) {
      return 6;
    } else if (RobotContainer.currentRobotState == RobotConstants.AMP) {
      return 8;
    } else if (RobotContainer.currentRobotState == RobotConstants.FEEDING) {
      return 10;
    } else if (RobotContainer.currentRobotState == RobotConstants.SOURCE_INTAKE) {
      return 12;
    } else if (RobotContainer.currentRobotState == RobotConstants.STATIONARY_SHOOTING) {
      return 14;
    } else if (RobotContainer.currentRobotState == RobotConstants.PRESET_SHOOTING) {
      return 16;
    } else {
      return 0;
    } 
  }
}
