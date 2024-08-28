// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Intake;

import com.ma5951.utils.StateControl.Subsystems.StateControlledSubsystem;

import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.RobotControl.SuperStructure;
import frc.robot.Subsystem.Arm.Arm;
import frc.robot.Subsystem.Intake.IOs.IntakeIO;

public class Intake extends StateControlledSubsystem {
  private static Intake intake;

  private IntakeIO intakeIO =  IntakeConstants.getIntakeIO();

  public Intake() {
    super(IntakeConstants.SYSTEM_STATES , "Intake");
    intakeIO.setNutralMode(false);
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

  public void setVoltage(double voltage) {
    intakeIO.setVoltage(voltage);
  }

  public void setPower(double power) {
    setVoltage(power * 12);
  }

  @Override
  public boolean canMove() {
      if ((RobotContainer.currentRobotState == RobotConstants.INTAKE && Arm.getInstance().atPoint() 
      && !SuperStructure.getInstance().isNote())||
          (RobotContainer.currentRobotState == RobotConstants.EJECT)) {
          return true;
      } else {
          return false;
      }
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
  }
}
