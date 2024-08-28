// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Shooter;

import com.ma5951.utils.DashBoard.MAShuffleboard;
import com.ma5951.utils.Logger.LoggedBool;
import com.ma5951.utils.Logger.LoggedDouble;
import com.ma5951.utils.StateControl.Subsystems.StateControlledSubsystem;

import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.RobotControl.SuperStructure;
import frc.robot.Subsystem.Shooter.IOs.ShooterIO;
import frc.robot.Utils.ShootingParameters;

public class Shooter extends StateControlledSubsystem {
  private static Shooter shooter;

  private ShooterIO shooterIO = ShooterConstants.getShooterIO();
  private double rightSetPoint;
  private double leftSetPoint;
  private MAShuffleboard board;

  private LoggedDouble leftSpeed;
  private LoggedDouble rightSpeed;
  private LoggedDouble leftSpeedAdust;
  private LoggedDouble rightSpeedAdust;
  private LoggedDouble leftSetPointLog;
  private LoggedDouble rightSetPointLog;
  private LoggedBool leftShotoerAtPoint;
  private LoggedBool rightShooterAtPoint;
  private LoggedBool shooterAtPoint;

  private Shooter() {
    super(ShooterConstants.SYSTEM_STATES , "Shooter");
    shooterIO.setShooterNutralMode(false);

    board = new MAShuffleboard("Shooter");
    board.addNum("Left Speed Adust", 0);
    board.addNum("Right Speed Adust", 0);

    leftSpeed = new LoggedDouble("/Subsystems/Shooter/Left Speed");
    rightSpeed = new LoggedDouble("/Subsystems/Shooter/Right Speed");
    leftSetPointLog = new LoggedDouble("/Subsystems/Shooter/Left Set Point");
    rightSetPointLog = new LoggedDouble("/Subsystems/Shooter/Right Set Point");
    leftShotoerAtPoint = new LoggedBool("/Subsystems/Shooter/Left At Point");
    rightShooterAtPoint = new LoggedBool("/Subsystems/Shooter/Right At Point");
    shooterAtPoint = new LoggedBool("/Subsystems/Shooter/At Point");
    leftSpeedAdust = new LoggedDouble("/Subsystems/Shooter/Left Speed Adjust");
    rightSpeedAdust = new LoggedDouble("/Subsystems/Shooter/Right Speed Adjust");
  }

  public double getCurrentDraw() {
    return ( shooterIO.getLeftCurrentDraw() + shooterIO.getRightCurrentDraw() ) / 2;
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
    return (leftAtPoint() && rightAtPoint()) && (Math.abs(getLeftSpeed() - getRightSpeed()) <= ShooterConstants.kTOLORANCE_BETWEEN_SIDES);
  }

  public void setRightVoltage(double voltage) {
    shooterIO.setRightVoltage(voltage);
  }

  public void setLeftVoltage(double voltage) {
    shooterIO.setLeftVoltage(voltage);
  }

  public void setVoltage(double voltage) {
    setRightVoltage(voltage);
    setLeftVoltage(voltage);
  }

  public void setRightPower(double power) {
    setRightVoltage(power * 12);
  }

  public void setLeftPower(double power) {
    setLeftVoltage(power * 12);
  }

  public void setPower(double power) {
    setVoltage(power * 12);
  }

  public void setShooterSpeeds(double leftRPM , double rightRPM) {
    setShootingParameterSpeeds(new ShootingParameters(leftRPM + board.getNum("Left Speed Adust"), + board.getNum("Right Speed Adust"), 0));
  }

  public void setShootingParameterSpeeds(ShootingParameters parameters) {
    rightSetPoint = parameters.getRightSpeed();
    leftSetPoint = parameters.getLeftSpeed();
    shooterIO.setRightSpeedSetPoint(rightSetPoint);
    shooterIO.setLeftSpeedSetPoint(leftSetPoint);
  }

  //Can Move
  private boolean StationaryShootCanMove() {
    return RobotContainer.currentRobotState == RobotConstants.STATIONARY_SHOOTING;
  }

  private boolean WarmingCanMove() {
    return RobotContainer.currentRobotState == RobotConstants.WARMING && SuperStructure.isInWarmUpZone();
  }

  private boolean FeedingCanMove() {
    return RobotContainer.currentRobotState == RobotConstants.FEEDING;
  }

  private boolean EjectCanMove() {
    return RobotContainer.currentRobotState == RobotConstants.EJECT;
  }

  private boolean SourceIntakeCanMove() {
    return RobotContainer.currentRobotState == RobotConstants.SOURCE_INTAKE;
  }


  @Override
  public boolean canMove() {
      return StationaryShootCanMove() || WarmingCanMove() || FeedingCanMove() || EjectCanMove() || SourceIntakeCanMove();
  }

  public static Shooter getInstance() {
    if (shooter == null) {
      shooter = new Shooter();  
    }
    return shooter;
  }

  @Override
  public void periodic() {
    super.periodic();
    shooterIO.updatePeriodic();

    leftSpeed.update(getLeftSpeed());
    rightSpeed.update(getRightSpeed());
    rightShooterAtPoint.update(rightAtPoint());
    leftShotoerAtPoint.update(leftAtPoint());
    shooterAtPoint.update(atPoint());
    leftSetPointLog.update(leftSetPoint);
    rightSetPointLog.update(rightSetPoint);
    leftSpeedAdust.update(board.getNum("Left Speed Adust"));
    rightSpeedAdust.update(board.getNum("Right Speed Adust"));
  }
}
