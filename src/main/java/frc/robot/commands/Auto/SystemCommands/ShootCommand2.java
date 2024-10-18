// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.SystemCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotConstants;
import frc.robot.Subsystem.Arm.Arm;
import frc.robot.Subsystem.Feeder.Feeder;
import frc.robot.Subsystem.Shooter.Shooter;
import frc.robot.Utils.ShootingParameters;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootCommand2 extends ParallelDeadlineGroup {
  /** Creates a new ShootCommand2. */
  public ShootCommand2() {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new SequentialCommandGroup(
      new InstantCommand(() -> Arm.getInstance().setAutoSetPoint(() -> RobotConstants.SUPER_STRUCTURE.getShootingPrameters().getArmAngle())),
      new WaitUntilCommand(() -> Shooter.getInstance().atPoint()),
      new WaitUntilCommand(0.2),
      new WaitUntilCommand(() -> Arm.getInstance().atPoint()),
      //new WaitUntilCommand(0.2),
      new InstantCommand(() -> Feeder.getInstance().turnOnForward()),
      new WaitUntilCommand(() -> !RobotConstants.SUPER_STRUCTURE.isNoteInShooter()),
      new InstantCommand(() -> Feeder.getInstance().turnOffFeeder()))
    );
    addCommands(
    );
  }
}
