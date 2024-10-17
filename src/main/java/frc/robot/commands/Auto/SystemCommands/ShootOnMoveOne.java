
package frc.robot.commands.Auto.SystemCommands;

import java.util.function.Supplier;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotConstants;
import frc.robot.Subsystem.Arm.Arm;
import frc.robot.Subsystem.Arm.ArmConstants;
import frc.robot.Subsystem.Feeder.Feeder;
import frc.robot.Subsystem.Intake.Intake;
import frc.robot.Subsystem.Shooter.Shooter;
import frc.robot.Utils.ShootingParameters;

public class ShootOnMoveOne extends ParallelDeadlineGroup {
  private static Supplier<Double> armAngle = () -> RobotConstants.SUPER_STRUCTURE.getShootingPrameters().getArmAngle() - 6;//-8


  public ShootOnMoveOne() {
    super(new SequentialCommandGroup(
      new InstantCommand(() -> Arm.getInstance().setAutoSetPoint(armAngle)),
      new InstantCommand(() -> Shooter.getInstance().setAutoShootingParameters(() -> new ShootingParameters(5800, 6000, 0, 0))),
      new WaitUntilCommand(() -> Shooter.getInstance().getRightSpeed() > 600),
      new WaitUntilCommand(() -> Math.abs(Arm.getInstance().getArmPosition() - Arm.getInstance().getAutoSetPoint()) <= ArmConstants.kTOLORANCE * 3),
      new InstantCommand(() -> Feeder.getInstance().turnOnForward()),
      new WaitUntilCommand(() -> !RobotConstants.SUPER_STRUCTURE.isNoteInShooter()))
    );

    addCommands(
  
    );
  }
}
