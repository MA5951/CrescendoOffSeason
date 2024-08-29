// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.DeafultCommands;

import com.ma5951.utils.StateControl.Commands.RobotFunctionStatesCommand;

import frc.robot.RobotContainer;
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
        intake.turnOffIntke();
        break;
      case "INTAKING":
        intake.turnOnIntke();
        break;
      case "EJECTING":
        intake.turnOnEjectIntake();
        break;
      
      default:
        break;
    }
  }

  @Override
  public void ManuelLoop() {
    super.ManuelLoop();
    if (RobotContainer.driverController.getHID().getPOV() == 0) {
      intake.turnOnIntke();
    } else if (RobotContainer.driverController.getHID().getPOV() == 180) {
      intake.turnOnEjectIntake();
    }
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
