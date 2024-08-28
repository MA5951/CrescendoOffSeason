// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.DeafultCommands;

import com.ma5951.utils.StateControl.Commands.RobotFunctionStatesCommand;

import frc.robot.Subsystem.Intake.Intake;

public class IntakeDeafultCommand extends RobotFunctionStatesCommand {
  private static Intake intake = Intake.getInstance();
  
  public IntakeDeafultCommand() {
    super(intake);
    addRequirements(intake);
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
    switch (intake.getCurrenState().getName()) {
      case "IDLE":
        Intake.getInstance().turnOffIntke();
        break;
      case "INTAKING":
        Intake.getInstance().turnOnIntke();
        break;
      case "EJECTING":
        Intake.getInstance().turnOnEjectIntake();
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
