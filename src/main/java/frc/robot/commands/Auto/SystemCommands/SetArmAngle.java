// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.SystemCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystem.Arm.Arm;
import frc.robot.Subsystem.Arm.ArmConstants;

public class SetArmAngle extends Command {
  private static Arm arm = Arm.getInstance();
  private Supplier<Double> setPoint; 

  public SetArmAngle(Supplier<Double> Angle) {
    setPoint = Angle;
  }

  @Override
  public void initialize() {
    //arm.setVoltage(0);
  }

  @Override
  public void execute() {
    arm.runSetPoint(setPoint.get());
  }

  @Override
  public void end(boolean interrupted) {
    arm.setVoltage(arm.getFeedForwardVoltage());
  }

  @Override
  public boolean isFinished() {
    return Math.abs(arm.getArmPosition() - (setPoint.get())) <= ArmConstants.kTOLORANCE;
  }
}
