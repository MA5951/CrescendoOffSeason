// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DeafultCommands;

import com.ma5951.utils.StateControl.Commands.RobotFunctionStatesCommand;

import frc.robot.RobotConstants;
import frc.robot.Subsystem.Shooter.Shooter;
import frc.robot.Subsystem.Shooter.ShooterConstants;
import frc.robot.Utils.ShootingParameters;

public class ShooterDeafultCommand extends RobotFunctionStatesCommand {
  private static Shooter shooter = Shooter.getInstance();

  public ShooterDeafultCommand() {
    super(shooter);
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
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
        //shooter.setVoltage(RobotConstants.WARM_VOLTAGE);
        
      //   if (RobotConstants.SUPER_STRUCTURE.getDistanceToTag() < 2.9) {
      //     shooter.setShootingParameterSpeeds(new ShootingParameters(3000, 3500, 0, 0));
      // } else {
      //     shooter.setShootingParameterSpeeds(new ShootingParameters(5500, 6000, 0, 0));
      // }
          shooter.setShootingParameterSpeeds(new ShootingParameters(4500, 5000, 0, 0));
        break;
      case "SHOOTING":
        shooter.setShootingParameterSpeeds(RobotConstants.SUPER_STRUCTURE.getShootingPrameters());
        break;
      case "FEEDING":
        shooter.setShootingParameterSpeeds(RobotConstants.SUPER_STRUCTURE.getFeedingPrameters());
        break;
      case "EJECTING":
        shooter.setShootingParameterSpeeds(new ShootingParameters(ShooterConstants.EJECTING_SPEED,
         ShooterConstants.EJECTING_SPEED, 0, 0));
        break;
      case "SOURCE_INTAKE":
        shooter.setVoltage(ShooterConstants.SOURCE_INTAKE_VOLTAGE);
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
      shooter.setManuelMode();

  }

  @Override
  public void AutoLoop() {
      super.AutoLoop();
      AutomaticLoop();
  } 
}
