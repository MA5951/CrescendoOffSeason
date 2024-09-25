// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of

package frc.robot.commands.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.Subsystem.Feeder.Feeder;
import frc.robot.Subsystem.Feeder.FeederConstants;
import frc.robot.Subsystem.Swerve.SwerveConstants;
import frc.robot.Subsystem.Swerve.SwerveSubsystem;

public class IntakeAutoDriveController extends Command {

  private ChassisSpeeds speed = new ChassisSpeeds();
  private CommandPS5Controller Controller;
  private double xSpeed;
  private double turningSpeed;

  public IntakeAutoDriveController(CommandPS5Controller controller) {
    this.Controller = controller;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    xSpeed = Controller.getLeftX();
    turningSpeed = Controller.getRightX();

    xSpeed = Math.abs(xSpeed) < 0.1 ? 0 : -xSpeed * SwerveConstants.DRIVER_XY_SCALER;
    turningSpeed = Math.abs(turningSpeed) < 0.1 ? 0 : -turningSpeed * SwerveConstants.DRIVER_XY_SCALER;

    if (RobotContainer.driverController.getHID().getR2Button()) {
      xSpeed = xSpeed * 0.4;
      turningSpeed = turningSpeed * 0.4;
    }

    if (RobotConstants.SUPER_STRUCTURE.isNote() || Feeder.getInstance().getTargetState() == FeederConstants.NOTE_ADJUSTING) {
        speed = new ChassisSpeeds(0.25 , xSpeed , turningSpeed );
          
    } else {
      speed = new ChassisSpeeds(-0.25 , xSpeed , turningSpeed );
    }
    // if (!RobotConstants.SUPER_STRUCTURE.isNote()) {
    //   speed = new ChassisSpeeds(-0.25 , xSpeed , turningSpeed );
    // }
      
  }

  public ChassisSpeeds getChassisSpeeds() {
    return speed;
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
