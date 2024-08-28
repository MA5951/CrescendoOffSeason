// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DeafultCommands;

import com.ma5951.utils.StateControl.Commands.RobotFunctionStatesCommand;

import frc.robot.RobotContainer;
import frc.robot.RobotControl.SuperStructure;
import frc.robot.Subsystem.Arm.Arm;
import frc.robot.Subsystem.Feeder.Feeder;

public class FeederDeafultCommand extends  RobotFunctionStatesCommand{
  private static Feeder feeder = Feeder.getInstance();
  private boolean releasForAmp = false;

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
          feeder.turnOffFeeder();
          break;
        case "FEEDING":
          feeder.turnOnFeeder();
          break;
        case "EJECTING":
          feeder.turnOnEjectFeeder();
          break;
        case "NOTE_ADJUSTING":
          break;
        case "AMP_REALES":
          if (SuperStructure.isNote() && Arm.getInstance().atPoint() && RobotContainer.driverController.getHID().getCircleButton()) {
            releasForAmp = true;
          } else if (releasForAmp) {
            if (SuperStructure.isNote())
            feeder.turnOnFeeder();
          } 
          break;
        default:
          break;
      }
  }

  @Override
  public void ManuelLoop() {
      super.ManuelLoop();
      if (RobotContainer.driverController.getHID().getPOV() == -90) {
        feeder.turnOnFeeder();
      } else if (RobotContainer.driverController.getHID().getPOV() == 90) {
        feeder.turnOnEjectFeeder();
      }
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
