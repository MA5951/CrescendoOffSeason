// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Arm.IOs;

/** Add your docs here. */
public interface ArmIO {

    double getCurrentDraw();

    double getAppliedVolts();

    double getMotorTemp();

    double getVelocity();

    double getPosition();

    void resetPosition(double newPose);

    void setNutralMode(boolean isBrake);

    void setAngleSetPoint(double angleSetPoint);

    void setVoltage();

    void updatePeriodic();
}
