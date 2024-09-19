// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DeafultCommands;

import com.ma5951.utils.StateControl.Commands.RobotFunctionStatesCommand;

import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.Subsystem.Arm.Arm;
import frc.robot.Subsystem.Arm.ArmConstants;

public class ArmDeafultCommand extends RobotFunctionStatesCommand {
  private static Arm arm = Arm.getInstance();
  
  public ArmDeafultCommand() {
    super(arm);
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    super.execute();
  }

  @Override
  public void end(boolean interrupted) {
   arm.setVoltage(arm.getFeedForwardVoltage());
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void AutomaticLoop() {
    super.AutomaticLoop();
    switch (arm.getTargetState().getName()) {
      case "IDLE":
      if (arm.getLastState() == ArmConstants.HOME ||arm.getLastState() == ArmConstants.INTAKE ) {
        arm.setVoltage(-arm.getFeedForwardVoltage() * 1);
      } else {
        arm.setVoltage(-arm.getFeedForwardVoltage());
      }
        break;
      case "FOLLOW_SPEAKER":
        // if (RobotConstants.SUPER_STRUCTURE.getDistanceToTag() > 3.5) {
        //   arm.runSetPoint(36);
        // } else {
        //   arm.runSetPoint(RobotConstants.SUPER_STRUCTURE.getShootingPrameters().getArmAngle());
        // }
        arm.runSetPoint(RobotConstants.SUPER_STRUCTURE.getShootingPrameters().getArmAngle());
        break;
      case "SOURCE_INTAKE":
        arm.runSetPoint(ArmConstants.SOURCE_INTAKE_POSE);
        break;
      case "PRESET_SHOOTING":
        arm.runSetPoint(RobotConstants.SUPER_STRUCTURE.getPRESETParameters().getArmAngle());
        break;
      case "AMP":
        arm.runSetPoint(ArmConstants.AMP_POSE);
        break;
      case "INTAKE":
        if (arm.atPoint() && arm.getSetPoint() == ArmConstants.INTAKE_POSE && RobotContainer.currentRobotState != RobotConstants.INTAKE) {
          arm.setTargetState(ArmConstants.IDLE);
        } else {
          arm.runSetPoint(ArmConstants.INTAKE_POSE);
        }
        break;
      case "HOME":
        if (arm.getLimit()) {
          arm.resetPosition(ArmConstants.ZERO_POSE);
          arm.setTargetState(ArmConstants.IDLE);
        } else if (arm.getArmPosition() > ArmConstants.ACTIVE_HOME_LIMIT_ANGLE) {
          arm.runSetPoint(ArmConstants.ACTIVE_HOME_LIMIT_ANGLE);
        } else {
          arm.setVoltage(ArmConstants.HOME_VOLTAGE);
        }
        break;
      default:
        break;
    }
  }

  @Override
  public void CAN_MOVE() {
    super.CAN_MOVE();
  }

  @Override
    public void CANT_MOVE() {
        super.CANT_MOVE();
        arm.setVoltage(arm.getFeedForwardVoltage());
    }

  @Override
  public void ManuelLoop() {
      super.ManuelLoop();
      double controllerMult = Math.abs(RobotContainer.oporatorController.getHID().getRightY()) < 0.1 ? arm.getFeedForwardVoltage() : (RobotContainer.oporatorController.getHID().getRightY() * -1 )
      * ArmConstants.MANUEL_VOLTAGE_LIMIT;
      arm.setVoltage(controllerMult);
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
