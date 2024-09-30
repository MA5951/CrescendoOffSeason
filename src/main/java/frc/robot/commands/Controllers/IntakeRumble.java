
package frc.robot.commands.Controllers;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;


public class IntakeRumble extends SequentialCommandGroup {

  public IntakeRumble() {
    addCommands(
      new InstantCommand(() -> RobotContainer.driverControllerRumble.getHID().setRumble(RumbleType.kBothRumble, 1)),
      new WaitCommand(0.5),
      new InstantCommand(() -> RobotContainer.driverControllerRumble.getHID().setRumble(RumbleType.kBothRumble, 0))
    );
  }
}
