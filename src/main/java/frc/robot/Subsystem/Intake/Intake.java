// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Intake;

import com.ma5951.utils.StateControl.Subsystems.StateControlledSubsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.RobotControl.RobotState;
import frc.robot.RobotControl.SuperStructure;
import frc.robot.Subsystem.Arm.Arm;
import frc.robot.Subsystem.Feeder.Feeder;
import frc.robot.Subsystem.Intake.IOs.IntakeIO;

public class Intake extends StateControlledSubsystem {
  private static Intake intake;

  private IntakeIO intakeIO =  IntakeConstants.getIntakeIO();

  public Intake() {
    super(IntakeConstants.SYSTEM_STATES);
    intakeIO.setNutralMode(false);
  }

  public void turnOnIntke() {
    intakeIO.setVoltage(IntakeConstants.INTAKE_POWER);
  }

  public void turnOnEjectIntake() {
    intakeIO.setVoltage(IntakeConstants.EJECT_POWER);
  }

  public void turnOffIntke() {
    intakeIO.setVoltage(0);
  }

  public void setVoltage(double voltage) {
    intakeIO.setVoltage(voltage);
  }

  public void setPower(double power) {
    intakeIO.setVoltage(power * 12);
  }

  @Override
  public int canMove() {
      if ((RobotState.getInstance().getRobotState() == RobotConstants.INTAKE && Arm.getInstance().atPoint() )||
          (RobotState.getInstance().getRobotState() == RobotConstants.EJECT)) {
          return 1;
      } else {
          return 0;
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
