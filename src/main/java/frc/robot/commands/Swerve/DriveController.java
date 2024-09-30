// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.Subsystem.Swerve.SwerveConstants;
import frc.robot.Subsystem.Swerve.SwerveSubsystem;

public class DriveController extends Command {

  private CommandPS5Controller Controller;
  private double xSpeed;
  private double ySpeed;
  private double turningSpeed;
  private ChassisSpeeds speed;
  private boolean angleLock = false;
  private PIDController anglePID = new PIDController(SwerveConstants.THATA_LOCK_KP, SwerveConstants.THATA_LOCK_KI, SwerveConstants.THATA_LOCK_KD);
  private double angleToLock;

  public DriveController(CommandPS5Controller controller) {
    Controller = controller;
    //anglePID.setTolerance(SwerveConstants.THATA_LOCK_PID_TOLORANCE);
    
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    xSpeed = Controller.getLeftX();
    ySpeed = Controller.getLeftY();
    turningSpeed = Controller.getRightX();

    xSpeed = Math.abs(xSpeed) < 0.1 ? 0 : -xSpeed * SwerveConstants.DRIVER_XY_SCALER;
    ySpeed = Math.abs(ySpeed) < 0.1 ? 0 : -ySpeed * SwerveConstants.DRIVER_XY_SCALER;
    turningSpeed = Math.abs(turningSpeed) < 0.1 ? 0 : -turningSpeed * SwerveConstants.DRIVER_THATA_SCALER;

    if (RobotContainer.driverController.getHID().getR2Button()) {
      xSpeed = xSpeed * 0.4;
      ySpeed = ySpeed * 0.4;
      turningSpeed = turningSpeed * 0.4;
    }

    // if (Math.abs(turningSpeed) > 0) {
    //   angleLock = false;
    // } else if (!angleLock && SwerveSubsystem.getInstance().getA().omegaRadiansPerSecond < 0.1 ) {
    //   angleToLock = SwerveSubsystem.getInstance().getFusedHeading();
    //   angleLock = true;
    // } else if (SwerveSubsystem.getInstance().getVelocity() > 1.5 && RobotContainer.currentRobotState != RobotConstants.INTAKE ){
    //   turningSpeed = anglePID.calculate(SwerveSubsystem.getInstance().getFusedHeading(), angleToLock);
    //   turningSpeed = Math.abs(turningSpeed) < SwerveConstants.THATA_LOCK_THRESHOLD ? 0 : turningSpeed;
    // }


    speed = ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed, turningSpeed,
                  new Rotation2d(
                    Math.toRadians((SwerveSubsystem.getInstance().getFusedHeading()
                     - SwerveSubsystem.getInstance().getOffsetAngle()))));

    //speed = new ChassisSpeeds(ySpeed, xSpeed, turningSpeed);

  }

  public void updateAngleToLock() {
    angleToLock = SwerveSubsystem.getInstance().getFusedHeading();
  }

  public ChassisSpeeds getChassisSpeeds() {
    return speed;
  }

  @Override
  public void end(boolean interrupted) {
    
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
