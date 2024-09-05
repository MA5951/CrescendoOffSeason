// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Arm.IOs;

import com.ma5951.utils.Logger.LoggedDouble;
import com.ma5951.utils.Utils.ConvUtil;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.RobotConstants;
import frc.robot.Subsystem.Arm.ArmConstants;

public class ArmIOSim implements ArmIO{

    
    private double appliedVolts = 0;
    private PIDController pidController = 
        new PIDController(ArmConstants.SIM_kP , ArmConstants.SIM_kI, ArmConstants.SIM_kD, frc.robot.RobotConstants.kDELTA_TIME);


    private SingleJointedArmSim armSim;
    private LoggedDouble motorTempLog;
    private LoggedDouble appliedVoltsLog;
    private LoggedDouble velocityLog;
    private LoggedDouble currentDrawLog;
    private LoggedDouble positionLog;

    public ArmIOSim() {
        armSim = new SingleJointedArmSim(DCMotor.getKrakenX60(1), ArmConstants.GEAR, SingleJointedArmSim.estimateMOI(0.38, 8.5), 0.38
        , ConvUtil.DegreesToRadians(ArmConstants.INTAKE_POSE), ConvUtil.DegreesToRadians(ArmConstants.AMP_POSE)
        , true, ConvUtil.DegreesToRadians(ArmConstants.INTAKE_POSE));

        motorTempLog = new LoggedDouble("/Subsystems/Arm/Sim/Motor Temp");
        appliedVoltsLog = new LoggedDouble("/Subsystems/Arm/Sim/Applied Voltage");
        velocityLog = new LoggedDouble("/Subsystems/Arm/Sim/Velocity");
        currentDrawLog = new LoggedDouble("/Subsystems/Arm/Sim/Motor Current");
        positionLog = new LoggedDouble("/Subsystems/Arm/Sim/Position");
    }

    public double getCurrentDraw() {
        return armSim.getCurrentDrawAmps();
    }

    public double getVelocity() {
        return 0;
    }

    public double getMotorTemp() {
        return 0;
    }
    
    public double getAppliedVolts() {
        return appliedVolts;
    }

    public double getPosition() {
        return ConvUtil.RadiansToDegrees(armSim.getAngleRads());
    }


    public void resetPosition(double newAngle) {
        armSim.setState(0 , 0);
    }

    public void setNutralMode(boolean isBrake) {
        
    }
    
    public void setAngleSetPoint(double angleSetPoint , double feedforward) {//TODO: feedforward
        setVoltage(pidController.calculate(getPosition(), angleSetPoint));
    }

    
    public void setVoltage(double volt) {
        appliedVolts = volt;
        armSim.setInputVoltage(volt);
        
    }
    
    public void updatePeriodic() {
        if (DriverStation.isDisabled()) {
            setVoltage(0);
        }
        
        armSim.update(RobotConstants.kDELTA_TIME);

        motorTempLog.update(getMotorTemp());
        appliedVoltsLog.update(getAppliedVolts());
        velocityLog.update(getVelocity());
        currentDrawLog.update(getCurrentDraw());
        positionLog.update(getPosition());
    }


    public void updatePIDValues(double Kp, double Ki, double Kd) {
        System.out.println("Kp: " + Kp + "Ki: " + Ki + "Kd: " + Kd);
    }
}
