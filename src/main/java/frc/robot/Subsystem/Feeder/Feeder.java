// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Feeder;

import com.ma5951.utils.DashBoard.MAShuffleboard;
import com.ma5951.utils.Logger.LoggedBool;
import com.ma5951.utils.StateControl.Subsystems.StateControlledSubsystem;

import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.RobotControl.SuperStructure;
import frc.robot.Subsystem.Arm.Arm;
import frc.robot.Subsystem.Feeder.IOs.FeederIO;
import frc.robot.Subsystem.Shooter.Shooter;

public class Feeder extends StateControlledSubsystem {
  private static Feeder feeder;

  private FeederIO feederIO =  FeederConstants.getFeederIO();

  private LoggedBool beamBrakerLog;
  private LoggedBool IntakeCanMoveLog;
  private LoggedBool EjectCanMoveLog;
  private LoggedBool FeedingCanLog;
  private LoggedBool StationaryShootCanMoveLog;
  private LoggedBool PresetShootingCanMoveLog;
  private LoggedBool CanMoveLog;
  private MAShuffleboard board;

  private Feeder() {
    super(FeederConstants.SYSTEM_STATES , "Feeder");
    feederIO.setNutralMode(true);
    beamBrakerLog = new LoggedBool("/Subsystems/Feeder/Is Note");
    IntakeCanMoveLog = new LoggedBool("/Subsystems/Feedr/Can Move/Intake");
    EjectCanMoveLog = new LoggedBool("/Subsystems/Feedr/Can Move/Eject");
    FeedingCanLog = new LoggedBool("/Subsystems/Feedr/Can Move/Feeding");
    StationaryShootCanMoveLog = new LoggedBool("/Subsystems/Feedr/Can Move/Stationary Shoot");
    PresetShootingCanMoveLog = new LoggedBool("/Subsystems/Feedr/Can Move/PresetShooting");
    CanMoveLog = new LoggedBool("/Subsystems/Feedr/Can Move");

    board = new MAShuffleboard("Feeder");
    board.addNum("Feeder Adjust", 0);
  }

  public double getAppliedVolts() {
    return feederIO.getAppliedVolts();
  }

  public double getCurrentDraw() {
    return feederIO.getCurrentDraw();
  }

  public boolean isNoteInFeeder() {
    return feederIO.getBeamBraker();
  }

  public void turnOnForward() {
    setVoltage(FeederConstants.FORWARD_VOLTAGE);
  }

  public void turnOnRevers() {
    setVoltage(FeederConstants.REVERS_VOLTAG);
  }

  public void turnOnAdjustForward() {
    setVoltage(FeederConstants.FORWARD_VOLTAGE);
  }

  public void turnOnAdjustRevers() {
    setVoltage(FeederConstants.REVERS_VOLTAG);
  }

  public void turnOffFeeder() {
    setVoltage(0 );
  }

  public void setVoltage(double voltage) {
    feederIO.setVoltage(voltage * board.getNum("Feeder Adjust"));
  }

  public void setPower(double power) {
    setVoltage(power * 12);
  }

  //Can Move
  private boolean IntakeCanMove(){
    return RobotContainer.currentRobotState == RobotConstants.INTAKE && Arm.getInstance().atPoint() ;
  }

  private boolean EjectCanMove(){
    return RobotContainer.currentRobotState == RobotConstants.EJECT;
  }

  private boolean FeedingCanMove() {
    return RobotContainer.currentRobotState == RobotConstants.FEEDING && Shooter.getInstance().atPoint() && Arm.getInstance().atPoint()
           && SuperStructure.getInstance().isHeadingForFeeding() ;
  }

  private boolean StationaryShootCanMove() {
    return RobotContainer.currentRobotState == RobotConstants.STATIONARY_SHOOTING && Shooter.getInstance().atPoint() && Arm.getInstance().atPoint() 
          && SuperStructure.getInstance().isHeadingForShooting() && !SuperStructure.getInstance().isRobotMoving() ;
  }

  private boolean AmpCanMove() {
    return RobotContainer.currentRobotState == RobotConstants.AMP && Arm.getInstance().atPoint();
  }

  private boolean PresetShootingCanMove() {
    return RobotContainer.currentRobotState == RobotConstants.PRESET_SHOOTING && Shooter.getInstance().atPoint() && Arm.getInstance().atPoint();
  }

  @Override
  public boolean canMove() {
      return IntakeCanMove()|| EjectCanMove() || FeedingCanMove()|| StationaryShootCanMove() || AmpCanMove() || PresetShootingCanMove();
  }

  public static Feeder getInstance() {
    if (feeder == null) {
        feeder = new Feeder();  
    }
    return feeder;
  }

  @Override
  public void periodic() {
    feederIO.updatePeriodic();

    super.periodic();
    beamBrakerLog.update(feederIO.getBeamBraker());
    IntakeCanMoveLog.update(IntakeCanMove());
    EjectCanMoveLog.update(EjectCanMove());
    FeedingCanLog.update(FeedingCanMove());
    StationaryShootCanMoveLog.update(StationaryShootCanMove());
    PresetShootingCanMoveLog.update(PresetShootingCanMove());
    CanMoveLog.update(canMove());

    board.addNum("Applied Volts", getAppliedVolts());
  }
}
