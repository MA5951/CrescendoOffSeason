// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DeafultCommands;

import com.ma5951.utils.StateControl.Commands.RobotFunctionStatesCommand;

import frc.robot.RobotConstants;
import frc.robot.RobotControl.SuperStructure;
import frc.robot.Subsystem.Shooter.Shooter;
import frc.robot.Subsystem.Shooter.ShooterConstants;

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
        shooter.setVoltage(0);
        break;
      case "WARM":
        shooter.setShootingParameterSpeeds(RobotConstants.WARM_SHOOTING_PARAMETERS);
        break;
      case "SHOOTING":
        shooter.setShootingParameterSpeeds(SuperStructure.getShootingPrameters());
        break;
      case "FEEDING":
        shooter.setShootingParameterSpeeds(SuperStructure.getFeedingPrameters());
        break;
      case "EJECTING":
        shooter.setVoltage(ShooterConstants.EJECTING_VOLTGE);
        break;
      case "SOURCE_INTAKE":
        shooter.setVoltage(ShooterConstants.SOURCE_INTAKE_VOLTAGE);
        break;
      case "PRESET_SHOOTING":
        shooter.setShootingParameterSpeeds(SuperStructure.getPRESETParameters());
        break;
      default:
        break;
    }
  }

  @Override
  public void ManuelLoop() {
      super.ManuelLoop();
      shooter.setManuelMode();
  }
}
