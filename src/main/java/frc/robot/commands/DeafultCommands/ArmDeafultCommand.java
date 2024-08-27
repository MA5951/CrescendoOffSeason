// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DeafultCommands;

import com.ma5951.utils.StateControl.Commands.RobotFunctionStatesCommand;

import frc.robot.RobotConstants;
import frc.robot.RobotControl.RobotState;
import frc.robot.Subsystem.Arm.Arm;
import frc.robot.Subsystem.Arm.ArmConstants;

public class ArmDeafultCommand extends RobotFunctionStatesCommand {
  private static Arm arm = Arm.getInstance();
  
  public ArmDeafultCommand() {
    super(arm);
    addRequirements(arm);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    super.execute();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void AutomaticLoop() {
    super.AutomaticLoop();
    switch (arm.getCurrenState().getName()) {
      case "IDLE":
        arm.setVoltage(0);
        break;
      case "FOLLOW_SPEAKER":
        //Follow speaker
        break;
      case "SOURCE_INTAKE":
        arm.runSetPoint(ArmConstants.SOURCE_INTAKE_POSE);
        break;
      case "AMP":
        arm.runSetPoint(ArmConstants.AMP_POSE);
        break;
      case "INTAKE":
        arm.runSetPoint(ArmConstants.INTAKE_POSE);
        break;
      case "HOME":
        if (arm.getCurrentDraw() > ArmConstants.HOME_CURRENTLIMIT) {
          arm.setVoltage(0);
          RobotState.getInstance().setRobotState(RobotConstants.IDLE);
        } else {
          arm.setVoltage(ArmConstants.HOME_VOLTAGE);
        }
        break;
      
      default:
        break;
    }
  }
}
