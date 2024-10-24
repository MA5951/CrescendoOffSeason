// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Feeder.IOs;


import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ma5951.utils.Logger.LoggedDouble;
import com.ma5951.utils.Utils.ConvUtil;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.PortMap;
import frc.robot.Subsystem.Feeder.FeederConstants;
import frc.robot.Subsystem.Intake.IntakeConstants;

public class FeederIOReal implements FeederIO {

    private TalonFX feederMotor;
    private TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    private DigitalInput feederBeamBraker;
    private StatusSignal<Double> currentDraw;
    private StatusSignal<Double> velocity;
    private StatusSignal<Double> motorTemp;
    private StatusSignal<Double> appliedVolts;

    private LoggedDouble motorTempLog;
    private LoggedDouble appliedVoltsLog;
    private LoggedDouble velocityLog;
    private LoggedDouble currentDrawLog;

    public FeederIOReal() {
        feederMotor = new TalonFX(PortMap.Feeder.FalconID , PortMap.CanBus.RioBus);
        feederBeamBraker = new DigitalInput(PortMap.Feeder.DIO_FeederSensor);

        configTalonFX();
        
        currentDraw = feederMotor.getStatorCurrent();
        velocity = feederMotor.getVelocity();
        motorTemp = feederMotor.getDeviceTemp();
        appliedVolts = feederMotor.getMotorVoltage();

        motorTempLog = new LoggedDouble("/Subsystems/Feeder/Real/Motor Temp");
        appliedVoltsLog = new LoggedDouble("/Subsystems/Feeder/Real/Applied Voltage");
        velocityLog = new LoggedDouble("/Subsystems/Feeder/Real/Feeder Velocity");
        currentDrawLog = new LoggedDouble("/Subsystems/Feeder/Real/Motor Current");
    }

    public void configTalonFX() {
        motorConfig.Feedback.SensorToMechanismRatio = IntakeConstants.Gear;

        motorConfig.Voltage.PeakForwardVoltage = 12;
        motorConfig.Voltage.PeakReverseVoltage = -12;

        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = FeederConstants.IsCurrentLimitEnabled;
        motorConfig.CurrentLimits.SupplyCurrentLimit = FeederConstants.ContinuesCurrentLimit;
        motorConfig.CurrentLimits.SupplyCurrentThreshold = FeederConstants.PeakCurrentLimit;
        motorConfig.CurrentLimits.SupplyTimeThreshold = FeederConstants.PeakCurrentTime;

        feederMotor.getConfigurator().apply(motorConfig);
    }

    public boolean getBeamBraker() {
        return !feederBeamBraker.get();
    }

    public double getCurrentDraw() {
        return currentDraw.getValueAsDouble();
    }

    public double getVelocity() {
        return ConvUtil.RPStoRPM(velocity.getValueAsDouble());
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
        feederMotor.getConfigurator().apply(motorConfig);
    }

    public void setVoltage(double volt) {
        feederMotor.setVoltage(volt);
    }

    public void updatePeriodic() {
        currentDraw.refresh();
        velocity.refresh();
        motorTemp.refresh();
        appliedVolts.refresh();

        motorTempLog.update(getMotorTemp());
        appliedVoltsLog.update(getAppliedVolts());
        velocityLog.update(getVelocity());
        currentDrawLog.update(getCurrentDraw());
    }



}
