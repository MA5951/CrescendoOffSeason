// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.StateControl.Subsystems;

import edu.wpi.first.wpilibj2.command.Command;

public class StateSystemDeafultCommand extends Command {
  
  private StateControlledSubsystem subsystem;

  public StateSystemDeafultCommand(StateControlledSubsystem subsystem) {
    this.subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    subsystem.getStateMeachin().SystemLoop();
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
