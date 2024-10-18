
package frc.robot.commands.Controllers;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Subsystem.LED.LED;


public class IntakeRumble extends SequentialCommandGroup {

  public IntakeRumble() {
    addCommands(
      new InstantCommand(() -> LED.getInstance().isIntake = true),
      new InstantCommand(() -> LED.getInstance().Green()),
      new InstantCommand(() -> RobotContainer.driverControllerRumble.getHID().setRumble(RumbleType.kBothRumble, 1)),
      new WaitCommand(0.1),
      new InstantCommand(() -> LED.getInstance().Red()),
      new InstantCommand(() -> RobotContainer.driverControllerRumble.getHID().setRumble(RumbleType.kBothRumble, 0)),
      new WaitCommand(0.1),
      new InstantCommand(() -> LED.getInstance().Green()),
      new InstantCommand(() -> RobotContainer.driverControllerRumble.getHID().setRumble(RumbleType.kBothRumble, 1)),
      new WaitCommand(0.1),
      new InstantCommand(() -> LED.getInstance().Red()),
      new InstantCommand(() -> RobotContainer.driverControllerRumble.getHID().setRumble(RumbleType.kBothRumble, 0)),
      new WaitCommand(0.1),
      new InstantCommand(() -> LED.getInstance().Green()),
      new InstantCommand(() -> RobotContainer.driverControllerRumble.getHID().setRumble(RumbleType.kBothRumble, 1)),
      new WaitCommand(0.1),
      new InstantCommand(() -> LED.getInstance().Red()),
      new InstantCommand(() -> RobotContainer.driverControllerRumble.getHID().setRumble(RumbleType.kBothRumble, 0)),
      new InstantCommand(() -> LED.getInstance().isIntake = false)
    );
  }
}
