// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystem.Arm.IOs.ArmIO;

public class Arm extends SubsystemBase {
  private static Arm arm;

  private ArmIO armIO = ArmConstants.getArmIO();
  private double setPoint = 0;

  public Arm() {
    armIO.setNutralMode(true);
  }

  public boolean atPoint() {
    return Math.abs(getArmPosition() - setPoint) <= ArmConstants.kTOLORANCE;
  }

  public double getArmPosition() {
    return armIO.getPosition();
  }

  public void setSetPoint(double setPoint) {
    this.setPoint = setPoint;
    armIO.setAngleSetPoint(setPoint);
  }

  public void setVoltage(double voltage) {
    armIO.setVoltage(voltage);
  }

  public void setPower(double power) {
    armIO.setVoltage(power * 12);
  }

  public static Arm getInstance() {
    if (arm == null) {
      arm = new Arm();  
    }
    return arm;
  }

  @Override
  public void periodic() {
    armIO.updatePeriodic();
  }
}
