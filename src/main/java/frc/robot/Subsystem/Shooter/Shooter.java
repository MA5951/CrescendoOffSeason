// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Shooter;

import com.ma5951.utils.Logger.LoggedBool;
import com.ma5951.utils.Logger.LoggedDouble;
import com.ma5951.utils.StateControl.Subsystems.StateControlledSubsystem;

import frc.robot.Subsystem.Shooter.IOs.ShooterIO;
import frc.robot.Utils.ShootingParameters;

public class Shooter extends StateControlledSubsystem {
  private static Shooter shooter;

  private ShooterIO shooterIO = ShooterConstants.getShooterIO();
  private double rightSetPoint;
  private double leftSetPoint;
  
  private LoggedDouble leftSpeed;
  private LoggedDouble rightSpeed;
  private LoggedDouble leftSetPointLog;
  private LoggedDouble rightSetPointLog;
  private LoggedBool leftShotoerAtPoint;
  private LoggedBool rightShooterAtPoint;
  private LoggedBool shooterAtPoint;

  public Shooter() {
    shooterIO.setShooterNutralMode(false);

    leftSpeed = new LoggedDouble("/Subsystems/Shooter/Left Speed");
    rightSpeed = new LoggedDouble("/Subsystems/Shooter/Right Speed");
    leftSetPointLog = new LoggedDouble("/Subsystems/Shooter/Left Set Point");
    rightSetPointLog = new LoggedDouble("/Subsystems/Shooter/Right Set Point");
    leftShotoerAtPoint = new LoggedBool("/Subsystems/Shooter/Left At Point");
    rightShooterAtPoint = new LoggedBool("/Subsystems/Shooter/Right At Point");
    shooterAtPoint = new LoggedBool("/Subsystems/Shooter/At Point");
  }

  public double getRightSpeed() {
    return shooterIO.getRightVelocity();
  }

  public double getLeftSpeed() {
    return shooterIO.getLeftVelocity();
  }

  public boolean leftAtPoint() {
    return Math.abs(getLeftSpeed() - leftSetPoint) <= ShooterConstants.kTOLORANCE;
  }

  public boolean rightAtPoint() {
    return Math.abs(getRightSpeed() - rightSetPoint) <= ShooterConstants.kTOLORANCE;
  }

  public boolean atPoint() {
    return leftAtPoint() && rightAtPoint();
  }

  public void setShooterSpeeds(double leftRPM , double rightRPM) {
    rightSetPoint = rightRPM;
    leftSetPoint = leftRPM;
    shooterIO.setRightSpeedSetPoint(rightRPM);
    shooterIO.setLeftSpeedSetPoint(leftRPM);
  }

  public void setShootingParameterSpeeds(ShootingParameters parameters) {
    setShooterSpeeds(parameters.getLeftSpeed() , parameters.getRightSpeed());
  }

  public static Shooter getInstance() {
    if (shooter == null) {
      shooter = new Shooter();  
    }
    return shooter;
  }

  @Override
  public void periodic() {
    shooterIO.updatePeriodic();

    leftSpeed.update(getLeftSpeed());
    rightSpeed.update(getRightSpeed());
    rightShooterAtPoint.update(rightAtPoint());
    leftShotoerAtPoint.update(leftAtPoint());
    shooterAtPoint.update(atPoint());
    leftSetPointLog.update(leftSetPoint);
    rightSetPointLog.update(rightSetPoint);
  }
}
