// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ma5951.utils.StateControl.Commands.RobotFunctionStatesCommand;
import com.ma5951.utils.Utils.ConvUtil;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotControl.RobotState;
import frc.robot.Subsystem.Arm.Arm;
import frc.robot.Subsystem.Arm.ArmConstants;
import frc.robot.Subsystem.Feeder.Feeder;
import frc.robot.Subsystem.Intake.Intake;
import frc.robot.Subsystem.Shooter.Shooter;
import frc.robot.commands.DeafultCommands.ArmDeafultCommand;
import frc.robot.commands.DeafultCommands.FeederDeafultCommand;
import frc.robot.commands.DeafultCommands.IntakeDeafultCommand;
import frc.robot.commands.DeafultCommands.ShooterDeafultCommand;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

 
  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    Intake.getInstance();
    Arm.getInstance();
    Feeder.getInstance();
    Shooter.getInstance();

    RobotState.getInstance().setRobotState(RobotConstants.IDLE);

    CommandScheduler.getInstance().setDefaultCommand(Arm.getInstance(), new ArmDeafultCommand());
    CommandScheduler.getInstance().setDefaultCommand(Feeder.getInstance(), new FeederDeafultCommand());
    CommandScheduler.getInstance().setDefaultCommand(Intake.getInstance(), new IntakeDeafultCommand());
    CommandScheduler.getInstance().setDefaultCommand(Shooter.getInstance(), new ShooterDeafultCommand());

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    RobotState.getInstance().setRobotState(RobotConstants.IDLE);
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
      
    RobotState.getInstance().setRobotState(RobotConstants.INTAKE);
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
