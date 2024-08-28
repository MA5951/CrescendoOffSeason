// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DeafultCommands;

import com.ma5951.utils.StateControl.Commands.RobotFunctionStatesCommand;

import frc.robot.Subsystem.Shooter.Shooter;

public class ShooterDeafultCommand extends RobotFunctionStatesCommand {
  private static Shooter shooter = Shooter.getInstance();

  public ShooterDeafultCommand() {
    super(shooter);
    addRequirements(shooter);
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
    switch (shooter.getCurrenState().getName()) {
      case "IDLE":
        
        break;
      case "WARM":
        
        break;
      case "SHOOTING":
        
        break;
      case "FEEDING":
        
        break;
      case "EJECTING":
        
        break;
      case "SOURCE_INTAKE":
        
        break;
      default:
        break;
    }
  }
}
