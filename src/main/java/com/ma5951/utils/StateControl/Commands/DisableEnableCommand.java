// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.StateControl.Commands;

import com.ma5951.utils.StateControl.RobotState.RobotStates;

import edu.wpi.first.wpilibj2.command.Command;

public class DisableEnableCommand extends Command {
  /** Creates a new DisableEnableCommand. */
  public DisableEnableCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (RobotStates.getRobotEnableState().getName()) {
            case "ROBOT_ENABLE":
                ENABLE_LOOP();
            default:
                DISABLE_LOOP();
            break;
    }
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public void ENABLE_LOOP() {

  }

  public void DISABLE_LOOP() {

  }
}
