// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Shooter.IOs;

import com.ma5951.utils.Logger.LoggedDouble;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.RobotConstants;
import frc.robot.Subsystem.Arm.ArmConstants;

public class ShooterIOSim {

    private DCMotorSim motor;
    private PIDController pidController;
    private double appliedVolts = 0;

    private LoggedDouble motorTempLog;
    private LoggedDouble appliedVoltsLog;
    private LoggedDouble velocityLog;
    private LoggedDouble currentDrawLog;

    public ShooterIOSim() {
        motor = new DCMotorSim(DCMotor.getKrakenX60(1), ArmConstants.GEAR, 0.025);

        motorTempLog = new LoggedDouble("/Subsystems/Arm/Sim/Motor Temp");
        appliedVoltsLog = new LoggedDouble("/Subsystems/Arm/Sim/Applied Voltage");
        velocityLog = new LoggedDouble("/Subsystems/Arm/Sim/Velocity");
        currentDrawLog = new LoggedDouble("/Subsystems/Arm/Sim/Motor Current");
    }

    public double getCurrentDraw() {
        return motor.getCurrentDrawAmps();
    }

    public double getVelocity() {
        return motor.getAngularVelocityRPM();
    }

    public double getMotorTemp() {
        return 0;
    }
    
    public double getAppliedVolts() {
        return appliedVolts;
    }

    public void setNutralMode(boolean isBrake) {
        
    }
    
    public void setSpeedSetPoint(double angleSetPoint) {
        setVoltage(pidController.calculate(getVelocity(), angleSetPoint));
    }

    
    public void setVoltage(double volt) {
        appliedVolts = volt;
        motor.setInputVoltage(volt);
    }
    
    public void updatePeriodic() {
        if (DriverStation.isDisabled()) {
            setVoltage(0);
        }
        
        motor.update(RobotConstants.kDELTA_TIME);

        motorTempLog.update(getMotorTemp());
        appliedVoltsLog.update(getAppliedVolts());
        velocityLog.update(getVelocity());
        currentDrawLog.update(getCurrentDraw());
    }
}
