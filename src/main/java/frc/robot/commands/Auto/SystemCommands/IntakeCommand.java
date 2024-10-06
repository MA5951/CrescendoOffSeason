// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.SystemCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotConstants;
import frc.robot.Subsystem.Arm.Arm;
import frc.robot.Subsystem.Arm.ArmConstants;
import frc.robot.Subsystem.Feeder.Feeder;
import frc.robot.Subsystem.Intake.Intake;

public class IntakeCommand extends SequentialCommandGroup {
  public IntakeCommand() {
    
    addCommands(
      new InstantCommand(() -> Arm.getInstance().setAutoSetPoint(() -> ArmConstants.INTAKE_POSE)),
      new InstantCommand(() -> Intake.getInstance().turnOnIntke())
      .alongWith(new InstantCommand(() -> Feeder.getInstance().turnOnForward())),
      new WaitUntilCommand(() -> RobotConstants.SUPER_STRUCTURE.isNoteInShooter()),
      new InstantCommand(() -> Intake.getInstance().turnOffIntke())
      .alongWith(new InstantCommand(() -> Feeder.getInstance().turnOffFeeder()))
    );
  }
}
