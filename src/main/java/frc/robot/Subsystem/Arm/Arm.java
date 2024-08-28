// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Arm;

import com.ma5951.utils.DashBoard.MAShuffleboard;
import com.ma5951.utils.Logger.LoggedBool;
import com.ma5951.utils.Logger.LoggedDouble;
import com.ma5951.utils.StateControl.Subsystems.StateControlledSubsystem;

import frc.robot.Subsystem.Arm.IOs.ArmIO;

public class Arm extends StateControlledSubsystem {
  private static Arm arm;

  private ArmIO armIO = ArmConstants.getArmIO();
  private double setPoint = 0;

  private LoggedBool atPointLog;
  private LoggedDouble setPointLog;
  private LoggedDouble armAngleLog;
  private LoggedDouble armOffset;
  private MAShuffleboard board;

  private Arm() {
    super(ArmConstants.SUBSYSTEM_STATES , "Arm");
    atPointLog = new LoggedBool("/Subsystems/Arm/At Point");
    setPointLog = new LoggedDouble("/Subsystems/Arm/Set Point");
    armAngleLog = new LoggedDouble("/Subsystems/Arm/Arm Angle");
    armOffset = new LoggedDouble("/Subsystems/Arm/Angle Offset");
    armIO.setNutralMode(true);
    board = new MAShuffleboard("Arm");
    board.addNum("Angle Offset", 0);
  }

  public double getCurrentDraw() {
    return armIO.getCurrentDraw();
  }

  public boolean atPoint() {
    return Math.abs(getArmPosition() - setPoint) <= ArmConstants.kTOLORANCE;
  }

  public double getArmPosition() {
    return armIO.getPosition();
  }

  public double getSetPoint() {
    return setPoint;
  }

  public void runSetPoint(double setPoint) {
    this.setPoint = setPoint;
    armIO.setAngleSetPoint(setPoint + board.getNum("Angle Offset"));
  }

  public double getVoltage() {
    return armIO.getAppliedVolts();
  }

  public void setVoltage(double voltage) {
    armIO.setVoltage(voltage);
  }

  public void setPower(double power) {
    armIO.setVoltage(power * 12);
  }


  //Can Move
  private boolean LimitsCanMove(){
    return (getArmPosition() > ArmConstants.LOWER_LIMIT && getArmPosition() < ArmConstants.UPPER_LIMIT) ||
     (getArmPosition() > ArmConstants.UPPER_LIMIT && getVoltage() < 0) ||
    (getArmPosition() < ArmConstants.LOWER_LIMIT && getVoltage() > 0);
  } 

  @Override
  public boolean canMove() {
    return LimitsCanMove();
  }

  public static Arm getInstance() {
    if (arm == null) {
      arm = new Arm();  
    }
    return arm;
  }

  @Override
  public void periodic() {
    super.periodic();
    armIO.updatePeriodic();
    atPointLog.update(atPoint());
    setPointLog.update(getSetPoint());
    armAngleLog.update(getArmPosition());
    armOffset.update(board.getNum("Angle Offset"));
  }
}
