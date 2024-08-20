// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Arm.IOs;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ma5951.utils.Utils.ConvUtil;

import edu.wpi.first.hal.simulation.DIODataJNI;
import frc.robot.PortMap;
import frc.robot.Subsystem.Arm.ArmConstants;
import frc.robot.Subsystem.Intake.IntakeConstants;

/** Add your docs here. */
public class ArmIOReal implements ArmIO {

    private TalonFX armMotor;
    private MotionMagicVoltage motionMagicControl = new MotionMagicVoltage(0);
    private TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    
    private StatusSignal<Double> currentDraw;
    private StatusSignal<Double> velocity;
    private StatusSignal<Double> motorTemp;
    private StatusSignal<Double> appliedVolts;
    private StatusSignal<Double> position;
    
    public ArmIOReal() {
        armMotor = new TalonFX(PortMap.Arm.KrakenArmMotor);

        currentDraw = armMotor.getStatorCurrent();
        velocity = armMotor.getVelocity();
        motorTemp = armMotor.getDeviceTemp();
        appliedVolts = armMotor.getMotorVoltage();
        position = armMotor.getPosition();
    }

    public void configMotor() {
        motorConfig.Feedback.SensorToMechanismRatio = ArmConstants.GEAR;
        
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        motorConfig.Slot0.kP = ArmConstants.kP;
        motorConfig.Slot0.kI = ArmConstants.kI;
        motorConfig.Slot0.kD = ArmConstants.kD;
        motorConfig.Slot0.kS = ArmConstants.kS;
        motorConfig.Slot0.kA = ArmConstants.kA;
        motorConfig.Slot0.kV = ArmConstants.kV;

        motorConfig.MotionMagic.MotionMagicAcceleration = ArmConstants.kACCELERATION;
        motorConfig.MotionMagic.MotionMagicCruiseVelocity = ArmConstants.kCRUSIE_VELOCITY;
        motorConfig.MotionMagic.MotionMagicJerk = ArmConstants.kJERK;

        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = ArmConstants.IsCurrentLimitEnabled;
        motorConfig.CurrentLimits.SupplyCurrentLimit = ArmConstants.ContinuesCurrentLimit;
        motorConfig.CurrentLimits.SupplyCurrentThreshold = ArmConstants.PeakCurrentLimit;
        motorConfig.CurrentLimits.SupplyTimeThreshold = ArmConstants.PeakCurrentTime;

        armMotor.getConfigurator().apply(motorConfig);
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
        return position.getValueAsDouble();
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
    
    public void setAngleSetPoint(double angleSetPoint) {
        armMotor.setControl(motionMagicControl.withPosition(angleSetPoint).withSlot(ArmConstants.CONTROL_SLOT)
        );
    }

    
    public void setVoltage(double volt) {
        armMotor.setVoltage(volt);
    }
    
    public void updatePeriodic() {
        currentDraw.refresh();
        velocity.refresh();
        motorTemp.refresh();
        appliedVolts.refresh();
        position.refresh();
    }
}
