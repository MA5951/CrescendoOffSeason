// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ma5951.utils.StateControl.StatesTypes.State;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Commands.DeafultCommands.ArmDeafultCommand;
import frc.robot.Commands.DeafultCommands.FeederDeafultCommand;
import frc.robot.Commands.DeafultCommands.IntakeDeafultCommand;
import frc.robot.Commands.DeafultCommands.ShooterDeafultCommand;
import frc.robot.Subsystem.Arm.Arm;
import frc.robot.Subsystem.Arm.ArmConstants;
import frc.robot.Subsystem.Feeder.Feeder;
import frc.robot.Subsystem.Feeder.FeederConstants;
import frc.robot.Subsystem.Intake.Intake;
import frc.robot.Subsystem.Intake.IntakeConstants;
import frc.robot.Subsystem.Shooter.Shooter;
import frc.robot.Subsystem.Shooter.ShooterConstants;

public class RobotContainer {
  public static State currentRobotState = RobotConstants.IDLE;
  public static State lastRobotState = RobotConstants.IDLE;


  public RobotContainer() {
    Intake.getInstance();
    Arm.getInstance();
    Feeder.getInstance();
    Shooter.getInstance();
    setDeafultCommands();
    configureBindings();
  }

  public void setIDLE() {
      lastRobotState = currentRobotState;
      currentRobotState = RobotConstants.IDLE;
      Arm.getInstance().setTargetState(ArmConstants.IDLE);
      Feeder.getInstance().setTargetState(FeederConstants.IDLE);
      Intake.getInstance().setTargetState(IntakeConstants.IDLE);
      Shooter.getInstance().setTargetState(ShooterConstants.IDLE);
  }

  public void setINTAKE() {
      currentRobotState = RobotConstants.IDLE;
      Arm.getInstance().setTargetState(ArmConstants.INTAKE);
      Intake.getInstance().setTargetState(IntakeConstants.INTAKING);
      Feeder.getInstance().setTargetState(FeederConstants.FEEDING);
      Shooter.getInstance().setTargetState(ShooterConstants.IDLE);
  }

  public void setEJECT() {
    lastRobotState = currentRobotState;
    currentRobotState = RobotConstants.EJECT;
    Arm.getInstance().setTargetState(ArmConstants.IDLE);
    Intake.getInstance().setTargetState(IntakeConstants.EJECTING);
    Feeder.getInstance().setTargetState(FeederConstants.EJECTING);
    Shooter.getInstance().setTargetState(ShooterConstants.EJECTING);
  }

  public void setWARMING() {
    lastRobotState = currentRobotState;
    currentRobotState = RobotConstants.WARMING;
    Arm.getInstance().setTargetState(ArmConstants.FOLLOW_SPEAKER);
    Intake.getInstance().setTargetState(IntakeConstants.IDLE);
    Feeder.getInstance().setTargetState(FeederConstants.IDLE);
    Shooter.getInstance().setTargetState(ShooterConstants.WARM);
  }

  public void setAMP() {
    lastRobotState = currentRobotState;
    currentRobotState = RobotConstants.AMP;
    Arm.getInstance().setTargetState(ArmConstants.AMP);
    Intake.getInstance().setTargetState(IntakeConstants.IDLE);
    Feeder.getInstance().setTargetState(FeederConstants.FEEDING);
    Shooter.getInstance().setTargetState(ShooterConstants.IDLE);
  }

  public void setFEEDING() {
    lastRobotState = currentRobotState;
    currentRobotState = RobotConstants.FEEDING;
    Arm.getInstance().setTargetState(ArmConstants.FEEDING);
    Intake.getInstance().setTargetState(IntakeConstants.IDLE);
    Feeder.getInstance().setTargetState(FeederConstants.FEEDING);
    Shooter.getInstance().setTargetState(ShooterConstants.FEEDING);
  }

  public void setSOURCE_INTAKE() {
    lastRobotState = currentRobotState;
    currentRobotState = RobotConstants.SOURCE_INTAKE;
    Arm.getInstance().setTargetState(ArmConstants.SOURCE_INTAKE);
    Intake.getInstance().setTargetState(IntakeConstants.IDLE);
    Feeder.getInstance().setTargetState(FeederConstants.EJECTING);
    Shooter.getInstance().setTargetState(ShooterConstants.SOURCE_INTAKE);
  }

  public void setSTATIONARY_SHOOTING() {
    lastRobotState = currentRobotState;
    currentRobotState = RobotConstants.STATIONARY_SHOOTING;
    Arm.getInstance().setTargetState(ArmConstants.FOLLOW_SPEAKER);
    Intake.getInstance().setTargetState(IntakeConstants.IDLE);
    Feeder.getInstance().setTargetState(FeederConstants.FEEDING);
    Shooter.getInstance().setTargetState(ShooterConstants.SHOOTING);
  }

  public void setPRESET_SHOOTING() {
    lastRobotState = currentRobotState;
    currentRobotState = RobotConstants.IDLE;
    Arm.getInstance().setTargetState(ArmConstants.FEEDING);
    Intake.getInstance().setTargetState(IntakeConstants.IDLE);
    Feeder.getInstance().setTargetState(FeederConstants.FEEDING);
    Shooter.getInstance().setTargetState(ShooterConstants.FEEDING);
  }

  public void setHOME() {
    lastRobotState = currentRobotState;
    currentRobotState = RobotConstants.IDLE;
    Arm.getInstance().setTargetState(ArmConstants.HOME);
    Intake.getInstance().setTargetState(IntakeConstants.IDLE);
    Feeder.getInstance().setTargetState(FeederConstants.IDLE);
    Shooter.getInstance().setTargetState(ShooterConstants.IDLE);
  }

  public State getRobotState() {
      return currentRobotState;
  }

  public State getLastRobotState() {
      return lastRobotState;
  }

  
  private void setDeafultCommands() {
    CommandScheduler.getInstance().setDefaultCommand(Arm.getInstance(), new ArmDeafultCommand());
    CommandScheduler.getInstance().setDefaultCommand(Feeder.getInstance(), new FeederDeafultCommand());
    CommandScheduler.getInstance().setDefaultCommand(Intake.getInstance(), new IntakeDeafultCommand());
    CommandScheduler.getInstance().setDefaultCommand(Shooter.getInstance(), new ShooterDeafultCommand());
  }

  private void configureBindings() {
    
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
