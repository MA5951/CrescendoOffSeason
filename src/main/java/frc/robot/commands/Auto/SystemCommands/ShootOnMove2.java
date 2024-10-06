// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.SystemCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotConstants;
import frc.robot.Subsystem.Arm.Arm;
import frc.robot.Subsystem.Feeder.Feeder;
import frc.robot.Subsystem.Shooter.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootOnMove2 extends SequentialCommandGroup {

  public ShootOnMove2() {
    addCommands(
      new WaitUntilCommand(() -> Arm.getInstance().atPoint()),
      new WaitUntilCommand(() -> Shooter.getInstance().atPoint()),
      new InstantCommand(() -> Feeder.getInstance().turnOnForward()),
      new WaitUntilCommand(() -> !RobotConstants.SUPER_STRUCTURE.isNoteInShooter()),
      new InstantCommand(() -> Feeder.getInstance().turnOffFeeder())
    );
  }
}
