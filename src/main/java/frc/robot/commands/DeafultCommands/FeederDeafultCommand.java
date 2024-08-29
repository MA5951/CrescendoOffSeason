// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.DeafultCommands;

import com.ma5951.utils.StateControl.Commands.RobotFunctionStatesCommand;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;
import frc.robot.RobotControl.SuperStructure;
import frc.robot.Subsystem.Arm.Arm;
import frc.robot.Subsystem.Feeder.Feeder;
import frc.robot.Subsystem.Feeder.FeederConstants;

public class FeederDeafultCommand extends  RobotFunctionStatesCommand{
  private static Feeder feeder = Feeder.getInstance();
  private boolean releasForAmp = false;
  private Timer timer = new Timer();

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
            timer.start();
            timer.reset();
          } else if (!timer.hasElapsed(FeederConstants.AMP_REALES_TIME_SECOUNDS)) {
            feeder.turnOnFeeder();
          } else {
            timer.stop();
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
