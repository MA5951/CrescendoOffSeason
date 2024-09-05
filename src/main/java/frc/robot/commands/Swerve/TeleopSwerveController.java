// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.Subsystem.Swerve.SwerveSubsystem;

public class TeleopSwerveController extends Command {
  
  private DriveController driveCommand;
  private AngleAdjustController angleAdjustCommand;
  private ChassisSpeeds driveControllerSpeeds;
  private ChassisSpeeds angleAdjustControllerSpeeds;

  private SwerveSubsystem swerve;
  private ChassisSpeeds robotSpeeds;
  
  public TeleopSwerveController(CommandPS5Controller controller) {
    swerve = SwerveSubsystem.getInstance();
    
    driveCommand = new DriveController(controller);
    angleAdjustCommand = new AngleAdjustController(() -> RobotConstants.SUPER_STRUCTURE.getSetPointForAline(), true);
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    driveCommand.initialize();
  }

  @Override
  public void execute() {
    driveCommand.execute();
    angleAdjustCommand.execute();
    driveControllerSpeeds = driveCommand.getChassisSpeeds();
    angleAdjustControllerSpeeds = angleAdjustCommand.getChassisSpeeds();

    if (RobotContainer.currentRobotState == RobotConstants.FEEDING || RobotContainer.currentRobotState == RobotConstants.STATIONARY_SHOOTING
    || RobotContainer.currentRobotState == RobotConstants.PRESET_SHOOTING) {
      robotSpeeds = new ChassisSpeeds(driveControllerSpeeds.vxMetersPerSecond , driveControllerSpeeds.vyMetersPerSecond, angleAdjustControllerSpeeds.omegaRadiansPerSecond);
    } else {
      robotSpeeds = driveControllerSpeeds;
    }

    swerve.drive(robotSpeeds);
  }

  @Override
  public void end(boolean interrupted) {
    driveCommand.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
