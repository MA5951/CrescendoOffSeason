// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Arm.IOs;

import com.ma5951.utils.Logger.LoggedDouble;
import com.ma5951.utils.Utils.ConvUtil;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.RobotConstants;
import frc.robot.Subsystem.Arm.ArmConstants;

public class ArmIOSim implements ArmIO{

    // private DCMotorSim  motor;
    // private PIDController pidController = 
    //     new PIDController(ArmConstants.SIM_kP , ArmConstants.SIM_kI, ArmConstants.SIM_kD, frc.robot.RobotConstants.kDELTA_TIME);
    // private double appliedVolts = 0;

    // private LoggedDouble motorTempLog;
    // private LoggedDouble appliedVoltsLog;
    // private LoggedDouble velocityLog;
    // private LoggedDouble currentDrawLog;
    // private LoggedDouble positionLog;

    public ArmIOSim() {
        // motor = new DCMotorSim(DCMotor.getKrakenX60(1), ArmConstants.GEAR, 0.025);

        // motorTempLog = new LoggedDouble("/Subsystems/Arm/Sim/Motor Temp");
        // appliedVoltsLog = new LoggedDouble("/Subsystems/Arm/Sim/Applied Voltage");
        // velocityLog = new LoggedDouble("/Subsystems/Arm/Sim/Velocity");
        // currentDrawLog = new LoggedDouble("/Subsystems/Arm/Sim/Motor Current");
        // positionLog = new LoggedDouble("/Subsystems/Arm/Sim/Position");
        System.out.println("GGGGGGGGGGGGGGGGGGGGGGGGGGG");
    }

    public double getCurrentDraw() {
        //return motor.getCurrentDrawAmps();
        return 0;
    }

    public double getVelocity() {
        //return motor.getAngularVelocityRPM();
        return 0;
    }

    public double getMotorTemp() {
        return 0;
    }
    
    public double getAppliedVolts() {
        //return appliedVolts;
        return 0;
    }

    public double getPosition() {
        //return ConvUtil.RotationsToDegrees(motor.getAngularPositionRotations());
        return 0;
    }


    public void resetPosition(double newAngle) {
       // motor.setState(0 , 0);
    }

    public void setNutralMode(boolean isBrake) {
        
    }
    
    public void setAngleSetPoint(double angleSetPoint) {
        //motor.setInputVoltage(pidController.calculate(getPosition(), angleSetPoint));
    }

    
    public void setVoltage(double volt) {
        //appliedVolts = volt;
        //setInputVoltage(volt);
    }
    
    public void updatePeriodic() {
        // if (DriverStation.isDisabled()) {
        //     setVoltage(0);
        // }
        
        // motor.update(RobotConstants.kDELTA_TIME);

        // motorTempLog.update(getMotorTemp());
        // appliedVoltsLog.update(getAppliedVolts());
        // velocityLog.update(getVelocity());
        // currentDrawLog.update(getCurrentDraw());
        // positionLog.update(getPosition());
    }
}
