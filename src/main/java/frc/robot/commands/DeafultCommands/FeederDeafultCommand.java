// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DeafultCommands;

import com.ma5951.utils.StateControl.Commands.RobotFunctionStatesCommand;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.RobotControl.SuperStructure;
import frc.robot.Subsystem.Arm.Arm;
import frc.robot.Subsystem.Feeder.Feeder;
import frc.robot.Subsystem.Feeder.FeederConstants;

public class FeederDeafultCommand extends  RobotFunctionStatesCommand{
  private static Feeder feeder = Feeder.getInstance();
  private Timer timer = new Timer();
  private boolean isAmpReales = false;
  private boolean isTimerReset = false;

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
        case "FORWARD":
          if (RobotContainer.currentRobotState == RobotConstants.INTAKE) {
            feeder.turnOnFeeder();
          } else if (RobotContainer.currentRobotState == RobotConstants.STATIONARY_SHOOTING || RobotContainer.currentRobotState == RobotConstants.PRESET_SHOOTING) {
            if (!isTimerReset) {
              timer.start();
              timer.reset();
              isTimerReset = true;
            } else if (!timer.hasElapsed(FeederConstants.SHOOTING_REALES_TIME_SECOUNDS)) {
              feeder.turnOnFeeder();
            } else {
              timer.stop();
              isTimerReset = false;
              RobotContainer.currentRobotState = RobotConstants.IDLE;
            }
          } 
          break;
        case "REVERSE":
          feeder.turnOnEjectFeeder();
          break;
        case "NOTE_ADJUSTING":
          break;
        case "AMP_REALES":
          if (SuperStructure.isNote() && Arm.getInstance().atPoint() && RobotContainer.driverController.getHID().getCircleButton()) {
            isAmpReales = true;
            timer.start();
            timer.reset();
          } else if ( isAmpReales && !timer.hasElapsed(FeederConstants.AMP_REALES_TIME_SECOUNDS)) {
            feeder.turnOnFeeder();
          } else {
            isAmpReales = false;
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
      if (RobotContainer.oporatorController.getHID().getPOV() == -90) {
        feeder.turnOnFeeder();
      } else if (RobotContainer.oporatorController.getHID().getPOV() == 90) {
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
