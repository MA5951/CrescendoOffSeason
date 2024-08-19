// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Intake.IOs;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.PortMap;

public class IntakeIOReal implements IntakeIO {

    private TalonFX intakeMotor;
    private StatusSignal<Double> currentDraw;
    private StatusSignal<Double> velocity;
    private StatusSignal<Double> motorTemp;
    private StatusSignal<Double> appliedVolts;

    public IntakeIOReal() {
        intakeMotor = new TalonFX(PortMap.Intake.KrakenIntakeMotor);

        currentDraw = intakeMotor.getStatorCurrent();
        velocity = intakeMotor.getVelocity();
        motorTemp = intakeMotor.getDeviceTemp();
        appliedVolts = intakeMotor.getMotorVoltage();
    }

    public double getCurrentDraw() {
        return currentDraw.getValueAsDouble();
    }

    public double getVelocity() {
        return velocity.getValueAsDouble();
    }

    public double getMotorTemp() {
        return motorTemp.getValueAsDouble();
    }
    
    public double getAppliedVolts() {
        return appliedVolts.getValueAsDouble();
    }

    public void setNutralMode(boolean isBrake) {
        
    }

    public void setVoltage(double volt) {
        intakeMotor.setVoltage(volt);
    }

    public void updatePeriodic() {
        currentDraw.refresh();
        velocity.refresh();
        motorTemp.refresh();
        appliedVolts.refresh();
    }



}
