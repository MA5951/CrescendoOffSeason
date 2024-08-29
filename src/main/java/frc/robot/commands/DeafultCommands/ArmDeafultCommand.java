// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DeafultCommands;

import com.ma5951.utils.StateControl.Commands.RobotFunctionStatesCommand;

import frc.robot.RobotContainer;
import frc.robot.RobotControl.SuperStructure;
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
        arm.runSetPoint(SuperStructure.getShootingPrameters().getArmAngle());
        break;
      case "SOURCE_INTAKE":
        arm.runSetPoint(ArmConstants.SOURCE_INTAKE_POSE);
        break;
      case "PRESET_SHOOTING":
        arm.runSetPoint(SuperStructure.getPRESETParameters().getArmAngle());
        break;
      case "AMP":
        arm.runSetPoint(ArmConstants.AMP_POSE);
        break;
      case "INTAKE":
        arm.runSetPoint(ArmConstants.INTAKE_POSE);
        break;
      case "HOME":
        if (arm.getCurrentDraw() > ArmConstants.HOME_CURRENTLIMIT) {
          arm.setTargetState(ArmConstants.IDLE);
          arm.resetPosition(ArmConstants.INTAKE_POSE);
        } else if (arm.getArmPosition() > ArmConstants.ACTIVE_HOME_LIMIT) {
          arm.runSetPoint(ArmConstants.INTAKE_POSE);
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
    arm.runSetPoint(arm.getArmPosition());
  }

  @Override
  public void ManuelLoop() {
      super.ManuelLoop();
      double controllerMult = Math.abs(RobotContainer.oporatorController.getHID().getLeftY()) < 0.2 ? 0 : RobotContainer.oporatorController.getHID().getLeftY() * -1;
      arm.setVoltage(ArmConstants.MANUEL_VOLTAGE_LIMIT * -controllerMult);
  }

  @Override
  public void AutoLoop() {
      super.AutoLoop();
  }

  @Override
  public void TestLoop() {
      super.TestLoop();
  }
}
