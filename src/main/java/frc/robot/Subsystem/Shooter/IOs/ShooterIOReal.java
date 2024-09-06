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
import com.ma5951.utils.Logger.LoggedDouble;
import com.ma5951.utils.Utils.ConvUtil;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.PortMap;
import frc.robot.Subsystem.Shooter.ShooterConstants;

/** Add your docs here. */
public class ShooterIOReal implements ShooterIO{

    private TalonFX motorLeft;
    private TalonFX motorRight;
    private VelocityVoltage controlLeft = new VelocityVoltage(0);
    private VelocityVoltage controlRight = new VelocityVoltage(0);
    private TalonFXConfiguration leftMotorConfig = new TalonFXConfiguration();
    private TalonFXConfiguration rightMotorConfig = new TalonFXConfiguration();

    private DigitalInput beambraker;

    private StatusSignal<Double> leftcurrentDraw;
    private StatusSignal<Double> leftvelocity;
    private StatusSignal<Double> leftmotorTemp;
    private StatusSignal<Double> leftappliedVolts;

    private StatusSignal<Double> rightcurrentDraw;
    private StatusSignal<Double> rightvelocity;
    private StatusSignal<Double> rightmotorTemp;
    private StatusSignal<Double> rightappliedVolts;

    private StatusSignal<Double> leftMotorError;
    private StatusSignal<Double> rightMotorError;

    private LoggedDouble LmotorTempLog;
    private LoggedDouble LappliedVoltsLog;
    private LoggedDouble LvelocityLog;
    private LoggedDouble LcurrentDrawLog;

    private LoggedDouble RmotorTempLog;
    private LoggedDouble RappliedVoltsLog;
    private LoggedDouble RvelocityLog;
    private LoggedDouble RcurrentDrawLog;

    private LoggedDouble LerrorLog;
    private LoggedDouble RerrorLog;

    public ShooterIOReal() {
        motorLeft = new TalonFX(PortMap.Shooter.FalconLeftMotor , PortMap.CanBus.RioBus);
        motorRight = new TalonFX(PortMap.Shooter.FalconRightMotor, PortMap.CanBus.RioBus);
        beambraker = new DigitalInput(PortMap.Shooter.DIO_ShooterSensor);

        configMotors();

        leftcurrentDraw = motorLeft.getStatorCurrent();
        leftvelocity = motorLeft.getVelocity();
        leftmotorTemp = motorLeft.getDeviceTemp();
        leftappliedVolts = motorLeft.getMotorVoltage();

        rightcurrentDraw = motorRight.getStatorCurrent();
        rightvelocity = motorRight.getVelocity();
        rightmotorTemp = motorRight.getDeviceTemp();
        rightappliedVolts = motorRight.getMotorVoltage();

        leftMotorError = motorLeft.getClosedLoopError();
        rightMotorError = motorRight.getClosedLoopError();

        LmotorTempLog = new LoggedDouble("/Subsystems/Shooter/Real/Left Motor/Motor Temp");
        LappliedVoltsLog = new LoggedDouble("/Subsystems/Shooter/Real/Left Motor/Motor Applied Volts");
        LvelocityLog = new LoggedDouble("/Subsystems/Shooter/Real/Left Motor/Motor Velocity");
        LcurrentDrawLog = new LoggedDouble("/Subsystems/Shooter/Real/Left Motor/Motor Current Draw");

        RmotorTempLog = new LoggedDouble("/Subsystems/Shooter/Real/Right Motor/Motor Temp");
        RappliedVoltsLog = new LoggedDouble("/Subsystems/Shooter/Real/Right Motor/Motor Applied Volts");
        RvelocityLog = new LoggedDouble("/Subsystems/Shooter/Real/Right Motor/Motor Velocity");
        RcurrentDrawLog = new LoggedDouble("/Subsystems/Shooter/Real/Right Motor/Motor Current Draw");

        RerrorLog = new LoggedDouble("/Subsystems/Shooter/Real/Right Motor/Error");
        LerrorLog = new LoggedDouble("/Subsystems/Shooter/Real/Left Motor/Error");

        
    }

    public void configMotors() {
        leftMotorConfig.Feedback.SensorToMechanismRatio = ShooterConstants.GEAR;
        leftMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        leftMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        leftMotorConfig.Slot0.kP = ShooterConstants.kP;
        leftMotorConfig.Slot0.kI = ShooterConstants.kI;
        leftMotorConfig.Slot0.kD = ShooterConstants.kD;


        leftMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = ShooterConstants.IsCurrentLimitEnabled;
        leftMotorConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.ContinuesCurrentLimit;
        leftMotorConfig.CurrentLimits.SupplyCurrentThreshold = ShooterConstants.PeakCurrentLimit;
        leftMotorConfig.CurrentLimits.SupplyTimeThreshold = ShooterConstants.PeakCurrentTime;


        rightMotorConfig.Feedback.SensorToMechanismRatio = ShooterConstants.GEAR;
        rightMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rightMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rightMotorConfig.Slot0.kP = ShooterConstants.kP;
        rightMotorConfig.Slot0.kI = ShooterConstants.kI;
        rightMotorConfig.Slot0.kD = ShooterConstants.kD;


        rightMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = ShooterConstants.IsCurrentLimitEnabled;
        rightMotorConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.ContinuesCurrentLimit;
        rightMotorConfig.CurrentLimits.SupplyCurrentThreshold = ShooterConstants.PeakCurrentLimit;
        rightMotorConfig.CurrentLimits.SupplyTimeThreshold = ShooterConstants.PeakCurrentTime;

    }

    public double getLeftError() {
        return leftMotorError.getValueAsDouble();
    }

    public double getRightError() {
        return rightMotorError.getValueAsDouble();
    }

    public boolean getBeamBraker() {
        return !beambraker.get();
    }

    public double getLeftCurrentDraw() {
        return leftcurrentDraw.getValueAsDouble();
    }

    public double getLeftVelocity() {
        return ConvUtil.RPStoRPM(leftvelocity.getValueAsDouble());
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

    public void setLeftSpeedSetPoint(double setPoint , double feedforward) {
        motorLeft.setControl(controlLeft.withVelocity(ConvUtil.RPMtoRPS(setPoint)).withSlot(ShooterConstants.CONTROL_SLOT)
        .withFeedForward(feedforward)
        .withLimitReverseMotion(getLeftVelocity() > 0));
    }

    public void setLeftVoltage(double volt) {
        motorLeft.setVoltage(volt);
    }

    public double getRightCurrentDraw() {
        return rightcurrentDraw.getValueAsDouble();
    }

    public double getRightVelocity() {
        return ConvUtil.RPStoRPM(rightvelocity.getValueAsDouble());
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

    public void setRightSpeedSetPoint(double setPoint , double feedforward) {
        motorRight.setControl(controlRight.withVelocity(ConvUtil.RPMtoRPS(setPoint)).withSlot(ShooterConstants.CONTROL_SLOT)
        .withFeedForward(feedforward)
        .withLimitReverseMotion(getRightVelocity() > 0));
    }

    public void setRightVoltage(double volt) {
        motorRight.setVoltage(volt);
    }

    public void updatePIDValues(double Kp , double Ki , double Kd) {
        leftMotorConfig.Slot0.kP = Kp;
        leftMotorConfig.Slot0.kI = Ki;
        leftMotorConfig.Slot0.kD = Kd;

        rightMotorConfig.Slot0.kP = Kp;
        rightMotorConfig.Slot0.kI = Ki;
        rightMotorConfig.Slot0.kD = Kd;

        motorLeft.getConfigurator().apply(leftMotorConfig);
        motorRight.getConfigurator().apply(rightMotorConfig);

    
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
        rightMotorError.refresh();
        leftMotorError.refresh();

        LmotorTempLog.update(getLeftMotorTemp());
        LappliedVoltsLog.update(getLeftAppliedVolts());
        LvelocityLog.update(getLeftVelocity());
        LcurrentDrawLog.update(getLeftCurrentDraw());
        RmotorTempLog.update(getRightMotorTemp());
        RappliedVoltsLog.update(getRightAppliedVolts());
        RvelocityLog.update(getRightVelocity());
        RcurrentDrawLog.update(getRightCurrentDraw());
        RerrorLog.update(rightMotorError.getValueAsDouble());
        LerrorLog.update(leftMotorError.getValueAsDouble());
    }
}
