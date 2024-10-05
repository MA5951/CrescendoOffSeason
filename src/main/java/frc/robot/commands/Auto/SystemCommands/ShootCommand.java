// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.SystemCommands;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotConstants;
import frc.robot.Subsystem.Feeder.Feeder;
import frc.robot.Subsystem.Shooter.Shooter;
import frc.robot.Utils.ShootingParameters;


public class ShootCommand extends SequentialCommandGroup {
  public ShootCommand() {
    
    addCommands(
      new InstantCommand(() -> Shooter.getInstance().setShootingParameterSpeeds(new ShootingParameters(5000, 5000, 0, 0))),
      new SetArmAngle(() -> RobotConstants.SUPER_STRUCTURE.getShootingPrameters().getArmAngle()),
      new WaitUntilCommand(() -> Shooter.getInstance().atPoint()),
      new InstantCommand(() -> Feeder.getInstance().turnOnForward()),
      new WaitUntilCommand(() -> !RobotConstants.SUPER_STRUCTURE.isNoteInShooter()),
      new InstantCommand(() -> Feeder.getInstance().turnOffFeeder())
    );
  }
}
