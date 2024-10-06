
package frc.robot.commands.Auto.SystemCommands;

import java.util.function.Supplier;

import javax.print.DocFlavor.INPUT_STREAM;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotConstants;
import frc.robot.RobotControl.SuperStructure;
import frc.robot.Subsystem.Arm.Arm;
import frc.robot.Subsystem.Feeder.Feeder;
import frc.robot.Subsystem.Intake.Intake;
import frc.robot.Subsystem.Shooter.Shooter;
import frc.robot.Utils.ShootingParameters;

public class ShootOnMove extends ParallelDeadlineGroup {
  private static Supplier<Double> armAngle = () -> RobotConstants.SUPER_STRUCTURE.getShootingPrameters().getArmAngle() - 8;


  public ShootOnMove() {
    super(new SequentialCommandGroup(
      new InstantCommand(() -> Arm.getInstance().setAutoSetPoint(armAngle)),
      new InstantCommand(() -> Feeder.getInstance().turnOffFeeder()),
      new InstantCommand(() -> Shooter.getInstance().setShootingParameterSpeeds(new ShootingParameters(5000, 6000, 0, 0))),
      //new WaitUntilCommand(() -> Shooter.getInstance().getRightSpeed() > 2500),
      new WaitUntilCommand(() -> Arm.getInstance().atPoint()),
      new InstantCommand(() -> Feeder.getInstance().turnOnForward()),
      new WaitUntilCommand(() -> !RobotConstants.SUPER_STRUCTURE.isNoteInShooter()),
      new InstantCommand(() -> Arm.getInstance().setAutoSetPoint(() -> 30d)),
      new InstantCommand(() -> Intake.getInstance().turnOnIntke()),
      new WaitUntilCommand(() -> RobotConstants.SUPER_STRUCTURE.isNoteInShooter()),
      // new WaitUntilCommand(() -> RobotConstants.SUPER_STRUCTURE.isNoteInShooter()),
      new InstantCommand(() -> Feeder.getInstance().turnOffFeeder()),
      new InstantCommand(() -> Intake.getInstance().turnOffIntke()),
      new InstantCommand(() -> Arm.getInstance().setAutoSetPoint(() -> RobotConstants.SUPER_STRUCTURE.getShootingPrameters().getArmAngle())),
      new WaitUntilCommand(() -> Arm.getInstance().atPoint()),
      new InstantCommand(() -> Feeder.getInstance().turnOnForward()),
      new WaitUntilCommand(() -> !RobotConstants.SUPER_STRUCTURE.isNoteInShooter()),
      new InstantCommand(() -> Feeder.getInstance().turnOffFeeder()))
    );

    addCommands(
  
    );
  }
}
