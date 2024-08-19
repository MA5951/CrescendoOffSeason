// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystem.Intake.IOs.IntakeIO;

public class Intake extends SubsystemBase {

  private IntakeIO intakeIO =  IntakeConstants.getIntakeIO();

  public Intake() {
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
  public void periodic() {
    intakeIO.updatePeriodic();
  }
}
