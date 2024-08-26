// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Shooter.IOs;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ma5951.utils.Utils.ConvUtil;

import frc.robot.PortMap;
import frc.robot.Subsystem.Shooter.ShooterConstants;
import frc.robot.Utils.ShootingParameters;

/** Add your docs here. */
public class ShooterIOReal implements ShooterIO{

    private TalonFX motorLeft;
    private TalonFX motorRight;
    private VelocityVoltage controlLeft;
    private VelocityVoltage controlRight;
    private TalonFXConfiguration leftMotorConfig;
    private TalonFXConfiguration rightMotorConfig;

    private StatusSignal<Double> leftcurrentDraw;
    private StatusSignal<Double> leftvelocity;
    private StatusSignal<Double> leftmotorTemp;
    private StatusSignal<Double> leftappliedVolts;

    private StatusSignal<Double> rightcurrentDraw;
    private StatusSignal<Double> rightvelocity;
    private StatusSignal<Double> rightmotorTemp;
    private StatusSignal<Double> rightappliedVolts;

    public ShooterIOReal() {
        motorLeft = new TalonFX(PortMap.Shooter.FalconLeftMotor);
        motorRight = new TalonFX(PortMap.Shooter.FalconRightMotor);

        configMotors();

        leftcurrentDraw = motorLeft.getStatorCurrent();
        leftvelocity = motorLeft.getVelocity();
        leftmotorTemp = motorLeft.getDeviceTemp();
        leftappliedVolts = motorLeft.getMotorVoltage();

        rightcurrentDraw = motorRight.getStatorCurrent();
        rightvelocity = motorRight.getVelocity();
        rightmotorTemp = motorRight.getDeviceTemp();
        rightappliedVolts = motorRight.getMotorVoltage();


    }

    public void configMotors() {
        leftMotorConfig.Feedback.SensorToMechanismRatio = ShooterConstants.GEAR;
        leftMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        leftMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        leftMotorConfig.Slot0.kP = ShooterConstants.kP;
        leftMotorConfig.Slot0.kI = ShooterConstants.kI;
        leftMotorConfig.Slot0.kD = ShooterConstants.kD;
        leftMotorConfig.Slot0.kV = ShooterConstants.kV;
        leftMotorConfig.Slot0.kS = ShooterConstants.kS;

        leftMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = ShooterConstants.IsCurrentLimitEnabled;
        leftMotorConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.ContinuesCurrentLimit;
        leftMotorConfig.CurrentLimits.SupplyCurrentThreshold = ShooterConstants.PeakCurrentLimit;
        leftMotorConfig.CurrentLimits.SupplyTimeThreshold = ShooterConstants.PeakCurrentTime;


        rightMotorConfig.Feedback.SensorToMechanismRatio = ShooterConstants.GEAR;
        rightMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rightMotorConfig.Slot0.kP = ShooterConstants.kP;
        rightMotorConfig.Slot0.kI = ShooterConstants.kI;
        rightMotorConfig.Slot0.kD = ShooterConstants.kD;
        rightMotorConfig.Slot0.kV = ShooterConstants.kV;
        rightMotorConfig.Slot0.kS = ShooterConstants.kS;

        rightMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = ShooterConstants.IsCurrentLimitEnabled;
        rightMotorConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.ContinuesCurrentLimit;
        rightMotorConfig.CurrentLimits.SupplyCurrentThreshold = ShooterConstants.PeakCurrentLimit;
        rightMotorConfig.CurrentLimits.SupplyTimeThreshold = ShooterConstants.PeakCurrentTime;
    }

    public double getLeftCurrentDraw() {
        return leftcurrentDraw.getValueAsDouble();
    }

    public double getLeftVelocity() {
        return leftvelocity.getValueAsDouble();
    }

    public double getLeftMotorTemp() {
        return leftmotorTemp.getValueAsDouble();
    }

    public double getLeftAppliedVolts() {
        return leftappliedVolts.getValueAsDouble();
    }

    public void setLeftNutralMode(boolean isBrake) {
        if (isBrake) {
            leftMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        } else {
            leftMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        }
        motorLeft.getConfigurator().apply(leftMotorConfig);
    }

    public void setLeftSpeedSetPoint(double setPoint) {
        motorLeft.setControl(controlLeft.withVelocity(ConvUtil.RPMtoRPS(setPoint)).withSlot(ShooterConstants.CONTROL_SLOT));
    }

    public void setLeftVoltage(double volt) {
        motorLeft.setVoltage(volt);
    }

    public double getRightCurrentDraw() {
        return rightcurrentDraw.getValueAsDouble();
    }

    public double getRightVelocity() {
        return rightvelocity.getValueAsDouble();
    }

    public double getRightMotorTemp() {
        return rightmotorTemp.getValueAsDouble();
    }

    public double getRightAppliedVolts() {
        return rightappliedVolts.getValueAsDouble();
    }

    public void setRightNutralMode(boolean isBrake) {
        if (isBrake) {
            rightMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        } else {
            rightMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        }
        motorRight.getConfigurator().apply(rightMotorConfig);
    }

    public void setRightSpeedSetPoint(double setPoint) {
        motorRight.setControl(controlRight.withVelocity(ConvUtil.RPMtoRPS(setPoint)).withSlot(ShooterConstants.CONTROL_SLOT));
    }

    public void setRightVoltage(double volt) {
        motorRight.setVoltage(volt);
    }

    public void setShooterNutralMode(boolean isBrake) {
        setLeftNutralMode(isBrake);
        setRightNutralMode(isBrake);
    }    

    public void updatePeriodic() {
        leftcurrentDraw.refresh();
        leftvelocity.refresh();
        leftmotorTemp.refresh();
        leftappliedVolts.refresh();
        rightcurrentDraw.refresh();
        rightvelocity.refresh();
        rightmotorTemp.refresh();
        rightappliedVolts.refresh();
    }
}
