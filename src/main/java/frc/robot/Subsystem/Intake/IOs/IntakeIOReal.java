// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Intake.IOs;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import frc.robot.PortMap;
import frc.robot.Subsystem.Intake.IntakeConstants;

public class IntakeIOReal implements IntakeIO {

    private TalonFX intakeMotor;
    private TalonFXConfiguration motorConfig;
    private StatusSignal<Double> currentDraw;
    private StatusSignal<Double> velocity;
    private StatusSignal<Double> motorTemp;
    private StatusSignal<Double> appliedVolts;

    public IntakeIOReal() {
        intakeMotor = new TalonFX(PortMap.Intake.KrakenIntakeMotor);

        motorConfig = new TalonFXConfiguration();
        currentDraw = intakeMotor.getStatorCurrent();
        velocity = intakeMotor.getVelocity();
        motorTemp = intakeMotor.getDeviceTemp();
        appliedVolts = intakeMotor.getMotorVoltage();
    }

    public void configTalonFX() {
        motorConfig.Feedback.SensorToMechanismRatio = IntakeConstants.Gear;
        
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = IntakeConstants.IsCurrentLimitEnabled;
        motorConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.ContinuesCurrentLimit;
        motorConfig.CurrentLimits.SupplyCurrentThreshold = IntakeConstants.PeakCurrentLimit;
        motorConfig.CurrentLimits.SupplyTimeThreshold = IntakeConstants.PeakCurrentTime;

        intakeMotor.getConfigurator().apply(motorConfig);
    }

    public double getCurrentDraw() {
        return currentDraw.getValueAsDouble();
    }

    public double getVelocity() {
        return Units.rps;
    }

    public double getMotorTemp() {
        return motorTemp.getValueAsDouble();
    }
    
    public double getAppliedVolts() {
        return appliedVolts.getValueAsDouble();
    }

    public void setNutralMode(boolean isBrake) {
        if (isBrake) {
            motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        } else {
            motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        }
        intakeMotor.getConfigurator().apply(motorConfig);
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
