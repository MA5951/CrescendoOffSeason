// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Arm;

import java.util.function.Supplier;

import com.ma5951.utils.Logger.LoggedBool;
import com.ma5951.utils.Logger.LoggedDouble;
import com.ma5951.utils.Logger.LoggedPose3d;
import com.ma5951.utils.StateControl.StatesTypes.StatesConstants;
import com.ma5951.utils.StateControl.Subsystems.StateControlledSubsystem;
import com.ma5951.utils.Utils.ConvUtil;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystem.Arm.IOs.ArmIO;
import frc.robot.Subsystem.Feeder.Feeder;
import frc.robot.Subsystem.Feeder.FeederConstants;

public class Arm extends StateControlledSubsystem {
  private static Arm arm;

  private ArmIO armIO = ArmConstants.getArmIO();
  private double setPoint = 0;
  private Supplier<Double> autoSetPoint = () -> ArmConstants.INTAKE_POSE;


  private Pose3d armPosition;
  private LoggedBool atPointLog;
  private LoggedDouble setPointLog;
  private LoggedDouble armAngleLog;
  private LoggedDouble armOffset;
  private LoggedPose3d armPose3d;
  private LoggedBool CanMove;
  private LoggedDouble feedForawdLog;
  private LoggedBool LimitLog;


  private Arm() {
    super(ArmConstants.SUBSYSTEM_STATES , "Arm");
    armPosition = ArmConstants.SIM_ARM_OFFSET;
    atPointLog = new LoggedBool("/Subsystems/Arm/At Point");
    setPointLog = new LoggedDouble("/Subsystems/Arm/Set Point");
    armAngleLog = new LoggedDouble("/Subsystems/Arm/Arm Angle");
    armOffset = new LoggedDouble("/Subsystems/Arm/Angle Offset");
    armPose3d = new LoggedPose3d("/Subsystems/Arm/Position");
    CanMove = new LoggedBool("/Subsystems/Arm/Can Move");
    LimitLog = new LoggedBool("/Subsystems/Arm/Limit");
    feedForawdLog = new LoggedDouble("/Subsystems/Arm/FeedForward");
    armIO.setNutralMode(true);
    board.addNum("Angle Offset", 0);
    board.addNum("Shooting Angle Offset", 0);

    board.addCommand("Reset Pose", new InstantCommand(() -> resetPosition(ArmConstants.ZERO_POSE)));
    resetPosition(ArmConstants.ZERO_POSE);
  }

  public boolean getLimit() {
    return !armIO.getLimitSwitch();
  }

  public boolean isArmMoving() {
    return armIO.getVelocity() > ArmConstants.kARM_MOVING_THRSHOLD_RPM;
  }

  public double getFeedForwardVoltage() {
    return ((ArmConstants.MG * Math.sin(ConvUtil.DegreesToRadians(getArmPosition())) * ArmConstants.ARM_LENGTH) / ArmConstants.GEAR) / ArmConstants.kSTALL_TOURQE * 12;
}

  public double getCurrentDraw() {
    return armIO.getCurrentDraw();
  }

  public void resetPosition(double position) {
    armIO.resetPosition(position);
  }

  public boolean atPoint() {
    if (DriverStation.isAutonomous()) {
      return Math.abs(getArmPosition() - (autoSetPoint.get())) <= ArmConstants.kTOLORANCE * 2;
    } else {
      if (getTargetState() == ArmConstants.FOLLOW_SPEAKER) {
      return Math.abs(getArmPosition() - (setPoint + board.getNum("Shooting Angle Offset"))) <= ArmConstants.kTOLORANCE;
    } else {
      return Math.abs(getArmPosition() - (setPoint + board.getNum("Angle Offset"))) <= ArmConstants.kTOLORANCE;
    }
    }
  }

  public double getArmPosition() {
    return armIO.getPosition();
  }

  public double getSetPoint() {
    return setPoint;
  }

  public double getAutoSetPoint() {
    return autoSetPoint.get();
  }

  public void setAutoSetPoint(Supplier<Double> setPoint) {
    autoSetPoint = setPoint;
  }

  public void runSetPoint(double setPoint) {
    if (getTargetState() == ArmConstants.FOLLOW_SPEAKER) {
      this.setPoint = setPoint + board.getNum("Angle Offset");
      armIO.setAngleSetPoint(ConvUtil.DegreesToRotations(setPoint + board.getNum("Shooting Angle Offset") ) , getFeedForwardVoltage());
    } else if (DriverStation.isAutonomous()){
      armIO.setAngleSetPoint(ConvUtil.DegreesToRotations(setPoint) , getFeedForwardVoltage());
    } else {
      this.setPoint = setPoint + board.getNum("Angle Offset");
      armIO.setAngleSetPoint(ConvUtil.DegreesToRotations(setPoint) , getFeedForwardVoltage());
    }
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

  public void setSetPoint(double setPoint) {
    this.setPoint = setPoint;
  }


  //Can Move
  private boolean LimitsCanMove(){
    return (((getArmPosition() > ArmConstants.LOWER_LIMIT) ||
     (getArmPosition() > ArmConstants.UPPER_LIMIT && getVoltage() < 0) ||
    (getArmPosition() < ArmConstants.LOWER_LIMIT && getVoltage() > 0) )|| getSystemFunctionState() == StatesConstants.MANUEL 
    )&& Feeder.getInstance().getTargetState() !=  FeederConstants.NOTE_ADJUSTING || getTargetState() == ArmConstants.HOME && 
    Math.abs(getCurrentDraw()) < ArmConstants.CAN_MOVE_CURRENT_LIMIT;
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
    CanMove.update(canMove());
    feedForawdLog.update(getFeedForwardVoltage());
    LimitLog.update(getLimit());

    board.addNum("Set Point", getSetPoint());
    board.addNum("Current Pose", getArmPosition());
    board.addNum("Shooting Angle Offset", board.getNum("Shooting Angle Offset"));


    armPosition = new Pose3d(armPosition.getX(), armPosition.getY(), armPosition.getZ(), 
  new Rotation3d(0, ConvUtil.DegreesToRadians(-getArmPosition()), 0));
    armPose3d.update(armPosition);
  }
}

