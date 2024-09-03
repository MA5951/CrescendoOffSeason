// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DeafultCommands;

import com.ma5951.utils.StateControl.Commands.RobotFunctionStatesCommand;

import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.Subsystem.Feeder.Feeder;
import frc.robot.Subsystem.Feeder.FeederConstants;

public class FeederDeafultCommand extends  RobotFunctionStatesCommand{
  private static Feeder feeder = Feeder.getInstance(); //TODO change to the constructor and cant be static//Cant
  private boolean isNoteBack = false;

  public FeederDeafultCommand() {
    super(feeder);
    addRequirements(feeder);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    super.execute();
  }

  @Override
  public void end(boolean interrupted) {
    feeder.setVoltage(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void AutomaticLoop() {
      super.AutoLoop();
      switch (feeder.getTargetState().getName()) {
        case "IDLE":
          feeder.turnOffFeeder();
          isNoteBack = false;
          break;
        case "FORWARD":
          if (RobotContainer.currentRobotState == RobotConstants.INTAKE || RobotContainer.currentRobotState == RobotConstants.EJECT) {
            feeder.turnOnForward();
          } else if (RobotContainer.currentRobotState == RobotConstants.STATIONARY_SHOOTING || RobotContainer.currentRobotState == RobotConstants.PRESET_SHOOTING) {
              feeder.turnOnForward();
          } 
          break;
        case "REVERSE":
          if (RobotContainer.currentRobotState == RobotConstants.AMP && RobotContainer.driverController.getHID().getCircleButton()) {
              feeder.turnOnRevers();
            } else {
            feeder.turnOffFeeder();
            }
          break;
        case "NOTE_ADJUSTING":
          if (RobotConstants.SUPER_STRUCTURE.isNoteInShooter() && !isNoteBack) {
            feeder.turnOnAdjustRevers();
          } else { 
            isNoteBack = true;
            if (!RobotConstants.SUPER_STRUCTURE.isNoteInShooter()) {
              feeder.turnOnAdjustForward();
              
            } else {
              feeder.setTargetState(FeederConstants.IDLE);
            }
          }
          break;
        default:
          break;
      }
  }

  @Override
  public void CANT_MOVE() {
      super.CANT_MOVE();
      feeder.turnOffFeeder();
      isNoteBack = false;
  }

  @Override
  public void ManuelLoop() {
      super.ManuelLoop();
      if (RobotContainer.oporatorController.getHID().getPOV() == -90) {
        feeder.turnOnForward();
      } else if (RobotContainer.oporatorController.getHID().getPOV() == 90) {
        feeder.turnOnRevers();
      }
  }

  @Override
  public void AutoLoop() {
      super.AutoLoop();
      //TODO add the AutomaticLoop
  }

  @Override
  public void TestLoop() {
      super.TestLoop();
  }
}
