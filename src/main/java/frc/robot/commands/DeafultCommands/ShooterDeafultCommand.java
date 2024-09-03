// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DeafultCommands;

import com.ma5951.utils.StateControl.Commands.RobotFunctionStatesCommand;

import frc.robot.RobotConstants;
import frc.robot.Subsystem.Shooter.Shooter;
import frc.robot.Subsystem.Shooter.ShooterConstants;

public class ShooterDeafultCommand extends RobotFunctionStatesCommand {
  private static Shooter shooter = Shooter.getInstance(); //TODO change to the constructor and cant be static//Cant

  public ShooterDeafultCommand() {
    super(shooter);
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    //TODO set the motor to work only between 0 and 1 / -1 to 0 
  }

  @Override
  public void execute() {
    super.execute();
  }

  @Override
  public void end(boolean interrupted) {
     shooter.setVoltage(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void AutomaticLoop() {
    super.AutomaticLoop();    
    switch (shooter.getTargetState().getName()) {
      case "IDLE":
        shooter.setVoltage(0);
        break;
      case "WARM":
        shooter.setShootingParameterSpeeds(RobotConstants.SUPER_STRUCTURE.getWarmingParameters());
      case "SHOOTING":
        shooter.setShootingParameterSpeeds(RobotConstants.SUPER_STRUCTURE.getShootingPrameters());
        break;
      case "FEEDING":
        shooter.setShootingParameterSpeeds(RobotConstants.SUPER_STRUCTURE.getFeedingPrameters());
        break;
      case "EJECTING":
        shooter.setVoltage(ShooterConstants.EJECTING_VOLTGE);
        break;
      case "SOURCE_INTAKE":
      //TODO cancel the init set
        shooter.setVoltage(ShooterConstants.SOURCE_INTAKE_VOLTAGE);
      //TODO cancel the cancel  
        break;
      case "PRESET_SHOOTING":
        shooter.setShootingParameterSpeeds(RobotConstants.SUPER_STRUCTURE.getPRESETParameters());
        break;
      default:
        break;
    }
  }

  @Override
  public void CANT_MOVE() {
      super.CANT_MOVE();
      shooter.setVoltage(0);
  }

  @Override
  public void ManuelLoop() {
      super.ManuelLoop();
      shooter.setManuelMode(); //change to be a command proparty
  }

  @Override
  public void AutoLoop() {
      super.AutoLoop();
      AutomaticLoop();
  } 
}
