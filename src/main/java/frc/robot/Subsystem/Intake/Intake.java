// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Intake;

import com.ma5951.utils.Logger.LoggedBool;
import com.ma5951.utils.Logger.LoggedDouble;
import com.ma5951.utils.StateControl.StatesTypes.StatesConstants;
import com.ma5951.utils.StateControl.Subsystems.StateControlledSubsystem;

import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.Subsystem.Arm.Arm;
import frc.robot.Subsystem.Feeder.Feeder;
import frc.robot.Subsystem.Feeder.FeederConstants;
import frc.robot.Subsystem.Intake.IOs.IntakeIO;
import frc.robot.Subsystem.Shooter.Shooter;

public class Intake extends StateControlledSubsystem {
  private static Intake intake;

  private IntakeIO intakeIO =  IntakeConstants.getIntakeIO();

  private LoggedDouble offsetLog;
  private LoggedBool IntakeCanMove;
  private LoggedBool EjectCanMove;
  private LoggedBool CanMove;
  
  private Intake() {
    super(IntakeConstants.SYSTEM_STATES , "Intake");
    intakeIO.setNutralMode(true);
    board.addNum("Intake Adjust" , 1);
    offsetLog = new LoggedDouble("/Subsystems/Intake/Offset");
    IntakeCanMove = new LoggedBool("/Subsystems/Intake/Can Move/Intake");
    EjectCanMove = new LoggedBool("/Subsystems/Intake/Can Move/Eject");
    CanMove = new LoggedBool("/Subsystems/Intake/Can Move");
  }

  public double getAppliedVolts() {
    return intakeIO.getAppliedVolts();
  }

  public double getCurrentDraw() {
    return intakeIO.getCurrentDraw();
  }

  public void turnOnIntke() {
    setVoltage(IntakeConstants.INTAKE_POWER);
  }

  public void turnOnEjectIntake() {
    setVoltage(IntakeConstants.EJECT_POWER);
  }

  public void turnOffIntke() {
    setVoltage(0);
  }

  public void setVoltage(double voltage ) {
    intakeIO.setVoltage(voltage * board.getNum("Intake Adjust"));
  }

  public void setPower(double power) {
    setVoltage(power * 12);
  }


  //Can Move
  private boolean IntakeCanMove(){
    return RobotContainer.currentRobotState == RobotConstants.INTAKE && Arm.getInstance().atPoint() && !RobotConstants.SUPER_STRUCTURE.isNoteInShooter() ;
  }

  private boolean EjectCanMove(){
    return RobotContainer.currentRobotState == RobotConstants.EJECT && Shooter.getInstance().atPoint();
  }

  @Override
  public boolean canMove() {
      return IntakeCanMove() || EjectCanMove() || getSystemFunctionState() == StatesConstants.MANUEL || Feeder.getInstance().getTargetState() ==  FeederConstants.NOTE_ADJUSTING;
  }

  public static Intake getInstance() {
    if (intake == null) {
        intake = new Intake();  
    }
    return intake;
  }

  @Override
  public void periodic() {
    super.periodic();
    intakeIO.updatePeriodic();
   
    board.addNum("Applied Volts", getAppliedVolts());

    offsetLog.update(board.getNum("Intake Adjust"));
    IntakeCanMove.update(IntakeCanMove());
    EjectCanMove.update(EjectCanMove());
    CanMove.update(canMove());
  }
}
