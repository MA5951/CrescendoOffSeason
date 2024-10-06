
package frc.robot.commands.Auto.SystemCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotConstants;
import frc.robot.Subsystem.Feeder.Feeder;
import frc.robot.Subsystem.Intake.Intake;


public class MircozAutomation extends SequentialCommandGroup {
  public MircozAutomation() {
    addCommands(
      new InstantCommand(() -> Intake.getInstance().turnOnEjectIntake())
      .alongWith(new InstantCommand(() -> Feeder.getInstance().turnOnRevers())),
      new WaitUntilCommand(() -> !RobotConstants.SUPER_STRUCTURE.isNoteInFeeder()),
      new InstantCommand(() -> Intake.getInstance().turnOnIntke())
      .alongWith(new InstantCommand(() -> Feeder.getInstance().turnOnForward())),
      new WaitUntilCommand(() -> RobotConstants.SUPER_STRUCTURE.isNoteInShooter()),
      new InstantCommand(() -> Intake.getInstance().turnOffIntke())
      .alongWith(new InstantCommand(() -> Feeder.getInstance().turnOffFeeder()))
    );
  }
}
