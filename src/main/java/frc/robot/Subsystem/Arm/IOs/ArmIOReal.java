// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Arm.IOs;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ma5951.utils.Logger.LoggedDouble;
import com.ma5951.utils.Utils.ConvUtil;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.PortMap;
import frc.robot.Subsystem.Arm.ArmConstants;


public class ArmIOReal implements ArmIO {

    private TalonFX armMotor;
    private MotionMagicVoltage motionMagicControl = new MotionMagicVoltage(0);
    private TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    private DigitalInput limit;
    
    private StatusSignal<Double> currentDraw;
    private StatusSignal<Double> velocity;
    private StatusSignal<Double> motorTemp;
    private StatusSignal<Double> appliedVolts;
    private StatusSignal<Double> position;
    private StatusSignal<Double> test;

    private LoggedDouble motorTempLog;
    private LoggedDouble appliedVoltsLog;
    private LoggedDouble velocityLog;
    private LoggedDouble currentDrawLog;
    private LoggedDouble positionLog;
    
    public ArmIOReal() {
        armMotor = new TalonFX(PortMap.Arm.KrakenArmMotor , PortMap.CanBus.CANivoreBus);
        limit = new DigitalInput(PortMap.Arm.DIO_LimitSwitch);


        motorTempLog = new LoggedDouble("/Subsystems/Arm/Real/Motor Temp");
        appliedVoltsLog = new LoggedDouble("/Subsystems/Arm/Real/Applied Voltage");
        velocityLog = new LoggedDouble("/Subsystems/Arm/Real/Velocity");
        currentDrawLog = new LoggedDouble("/Subsystems/Arm/Real/Motor Current");
        positionLog = new LoggedDouble("/Subsystems/Arm/Real/Position");

        currentDraw = armMotor.getStatorCurrent();
        velocity = armMotor.getVelocity();
        motorTemp = armMotor.getDeviceTemp();
        appliedVolts = armMotor.getMotorVoltage();
        position = armMotor.getPosition();
        test = armMotor.getClosedLoopReference();

        configMotor();
    
    }

    public void configMotor() {
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        motorConfig.Feedback.SensorToMechanismRatio = ArmConstants.GEAR;

        motorConfig.Voltage.PeakForwardVoltage = 12;
        motorConfig.Voltage.PeakReverseVoltage = -12;
        
        
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        motorConfig.Slot0.kP = ArmConstants.kP;
        motorConfig.Slot0.kI = ArmConstants.kI;
        motorConfig.Slot0.kD = ArmConstants.kD;
        
        motorConfig.MotionMagic.MotionMagicAcceleration = ArmConstants.kACCELERATION;
        motorConfig.MotionMagic.MotionMagicCruiseVelocity = ArmConstants.kCRUSIE_VELOCITY;
        motorConfig.MotionMagic.MotionMagicJerk = ArmConstants.kJERK;

        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = ArmConstants.IsCurrentLimitEnabled;
        motorConfig.CurrentLimits.SupplyCurrentLimit = ArmConstants.ContinuesCurrentLimit;
        motorConfig.CurrentLimits.SupplyCurrentThreshold = ArmConstants.PeakCurrentLimit;
        motorConfig.CurrentLimits.SupplyTimeThreshold = ArmConstants.PeakCurrentTime;

        armMotor.getConfigurator().apply(motorConfig);
    }

    public boolean getReverseLimit() {
        return getPosition() < ArmConstants.LOWER_LIMIT;
    }

    public boolean getLimitSwitch() {
        return limit.get();
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

    public double getPosition() {
        return ConvUtil.RotationsToDegrees(position.getValueAsDouble());
    }


    public void resetPosition(double newAngle) {
        armMotor.setPosition(ConvUtil.DegreesToRotations(newAngle));
    }

    public void setNutralMode(boolean isBrake) {
        if (isBrake) {
            motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        } else {
            motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        }
        armMotor.getConfigurator().apply(motorConfig);
    }

    public void updatePIDValues(double Kp , double Ki , double Kd) {
        motorConfig.Slot0.kP = Kp;
        motorConfig.Slot0.kI = Ki;
        motorConfig.Slot0.kD = Kd;

        armMotor.getConfigurator().apply(motorConfig);
    }
    
    public void setAngleSetPoint(double angleSetPoint , double feedforward) {
        armMotor.setControl(motionMagicControl.withPosition(angleSetPoint).withSlot(ArmConstants.CONTROL_SLOT)
        .withFeedForward(feedforward)
        .withLimitReverseMotion(getReverseLimit()));        
    }

    
    public void setVoltage(double volt) {
        armMotor.setVoltage(volt);
    }
    
    public void updatePeriodic() {
        test.refresh();
        currentDraw.refresh();
        velocity.refresh();
        motorTemp.refresh();
        appliedVolts.refresh();
        position.refresh();

        motorTempLog.update(getMotorTemp());
        appliedVoltsLog.update(getAppliedVolts());
        velocityLog.update(getVelocity());
        currentDrawLog.update(getCurrentDraw());
        positionLog.update(getPosition());
        // System.out.println(test.getValueAsDouble());
    }
}
