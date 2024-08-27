// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DeafultCommands;

import com.ma5951.utils.StateControl.Commands.RobotFunctionStatesCommand;

import frc.robot.Subsystem.Feeder.Feeder;

public class FeederDeafultCommand extends  RobotFunctionStatesCommand{
  private static Feeder feeder = Feeder.getInstance();
  
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
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void AutomaticLoop() {
      super.AutoLoop();
      switch (feeder.getCurrenState().getName()) {
        case "IDLE":
          Feeder.getInstance().turnOffFeeder();
          break;
        case "FEEDING":
          Feeder.getInstance().turnOnFeeder();
          break;
        case "EJECTING":
          Feeder.getInstance().turnOnEjectFeeder();
          break;
        default:
          break;
      }
  }

  @Override
  public void ManuelLoop() {
      super.ManuelLoop();
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
