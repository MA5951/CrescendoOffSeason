
package frc.robot.commands.Auto.SystemCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystem.Feeder.Feeder;
import frc.robot.Subsystem.Shooter.Shooter;
import frc.robot.Utils.ShootingParameters;

public class Eject extends SequentialCommandGroup {
  public Eject() {
    addCommands(
      new InstantCommand(() -> Shooter.getInstance().setShootingParameterSpeeds(new ShootingParameters(5000, 5000, 0, 0))),
      new InstantCommand(() -> Feeder.getInstance().turnOnForward()),
      new WaitCommand(1)
    );
  }
}
