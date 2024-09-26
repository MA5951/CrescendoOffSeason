// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DeafultCommands;

import com.ma5951.utils.StateControl.Commands.RobotFunctionStatesCommand;

import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.Subsystem.Feeder.Feeder;
import frc.robot.Subsystem.Feeder.FeederConstants;
import frc.robot.Subsystem.Intake.Intake;
import frc.robot.Subsystem.Intake.IntakeConstants;
import frc.robot.Subsystem.Swerve.SwerveConstants;
import frc.robot.commands.Swerve.TeleopSwerveController;

public class FeederDeafultCommand extends  RobotFunctionStatesCommand{
  private static Feeder feeder = Feeder.getInstance(); 
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
          } else if ((RobotContainer.currentRobotState == RobotConstants.STATIONARY_SHOOTING && TeleopSwerveController.timeAtSetPoint.hasElapsed(SwerveConstants.TIME_AT_SET_POINT)) || RobotContainer.currentRobotState == RobotConstants.PRESET_SHOOTING
          ) {
              feeder.turnOnForward();
          } 
          break;
        case "REVERSE":
          if (RobotContainer.currentRobotState == RobotConstants.AMP && RobotContainer.driverController.getHID().getL2Button()) {
              feeder.turnOnRevers();
              RobotConstants.SUPER_STRUCTURE.updateAmpPose();
            } else if (RobotContainer.currentRobotState == RobotConstants.SOURCE_INTAKE){
              feeder.turnOnAdjustRevers();
            } else {
              feeder.turnOffFeeder();
            }
          break;
        case "NOTE_ADJUSTING":
          if (RobotContainer.lastRobotState == RobotConstants.INTAKE && RobotContainer.currentRobotState != RobotConstants.AMP) {
            if (RobotConstants.SUPER_STRUCTURE.isNoteInFeeder() && !isNoteBack) {
              feeder.turnOnRevers();
              Intake.getInstance().setTargetState(IntakeConstants.EJECTING);
            } else if (!RobotConstants.SUPER_STRUCTURE.isNoteInShooter()){
              isNoteBack = true;
              feeder.turnOnForward();
              Intake.getInstance().setTargetState(IntakeConstants.INTAKING);
            } else {
              feeder.setTargetState(FeederConstants.IDLE);
              Intake.getInstance().setTargetState(IntakeConstants.IDLE);
            }
          } else if (RobotContainer.lastRobotState == RobotConstants.SOURCE_INTAKE && RobotContainer.currentRobotState != RobotConstants.AMP) {
            if (!RobotConstants.SUPER_STRUCTURE.isNoteInShooter()) {
              feeder.turnOnForward();
            } else {
              feeder.setTargetState(FeederConstants.IDLE);
            }
          } else if (RobotContainer.currentRobotState == RobotConstants.AMP) {
              feeder.setTargetState(FeederConstants.REVERSE);
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
      // if (RobotContainer.oporatorController.getHID().getPOV() == 270) {
      //   feeder.turnOnForward();
      // } else if (RobotContainer.oporatorController.getHID().getPOV() == 90) {
      //   feeder.turnOnRevers();
      // } else {
      //   feeder.turnOffFeeder();
      // }

      if (RobotContainer.oporatorController.getHID().getSquareButton()) {
        feeder.turnOnForward();
      } else if (RobotContainer.oporatorController.getHID().getCircleButton()) {
        feeder.turnOnRevers();
      } else {
        feeder.turnOffFeeder();
      }
  }

  @Override
  public void AutoLoop() {
      super.AutoLoop();
      AutomaticLoop();
  }

  @Override
  public void TestLoop() {
      super.TestLoop();
  }
}
