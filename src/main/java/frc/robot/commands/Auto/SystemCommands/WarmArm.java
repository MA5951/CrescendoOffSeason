
package frc.robot.commands.Auto.SystemCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotConstants;
import frc.robot.Subsystem.Arm.Arm;
import frc.robot.Subsystem.Arm.ArmConstants;
import frc.robot.Subsystem.Feeder.Feeder;
import frc.robot.Subsystem.Intake.Intake;

public class WarmArm extends SequentialCommandGroup {
  public WarmArm() {
    addCommands(
      new InstantCommand(() -> Arm.getInstance().setAutoSetPoint(() -> ArmConstants.INTAKE_POSE)),
      new InstantCommand(() -> Intake.getInstance().turnOnIntke())
      .alongWith(new InstantCommand(() -> Feeder.getInstance().turnOnForward())),
      new WaitUntilCommand(() -> RobotConstants.SUPER_STRUCTURE.isNoteInShooter()),
      new InstantCommand(() -> Intake.getInstance().turnOffIntke())
      .alongWith(new InstantCommand(() -> Feeder.getInstance().turnOffFeeder())),
      new InstantCommand(() -> Arm.getInstance().setAutoSetPoint(
      () -> RobotConstants.SUPER_STRUCTURE.getShootingPrameters().getArmAngle()
    ))
    );
  }
}
