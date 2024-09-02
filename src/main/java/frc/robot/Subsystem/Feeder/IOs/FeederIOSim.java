// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Feeder.IOs;

import com.ma5951.utils.Logger.LoggedDouble;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.PortMap;
import frc.robot.RobotConstants;
import frc.robot.Subsystem.Feeder.FeederConstants;

/** Add your docs here. */
public class FeederIOSim implements FeederIO{

    private DCMotorSim motor;
    private DigitalInput feederBeamBraker;
    private double appliedVolts;

    private LoggedDouble motorTempLog;
    private LoggedDouble appliedVoltsLog;
    private LoggedDouble velocityLog;
    private LoggedDouble currentDrawLog;

    public FeederIOSim() {
        motor = new DCMotorSim(DCMotor.getFalcon500(1), FeederConstants.GEAR, 0.05);
        feederBeamBraker = new DigitalInput(PortMap.Feeder.DIO_FeederSensor);

        motorTempLog = new LoggedDouble("/Subsystems/Feeder/Sim/Motor Temp");
        appliedVoltsLog = new LoggedDouble("/Subsystems/Feeder/Sim/Applied Voltage");
        velocityLog = new LoggedDouble("/Subsystems/Feeder/Sim/Velocity");
        currentDrawLog = new LoggedDouble("/Subsystems/Feeder/Sim/Motor Current");

    }

    public boolean getBeamBraker() {
        return feederBeamBraker.get();
    }

    public double getCurrentDraw() {
        return motor.getCurrentDrawAmps();
    }

    public double getVelocity() {
      return  motor.getAngularVelocityRPM();
    }
    
    public double getMotorTemp() {
        return 0;
    }

    public double getAppliedVolts() {
       return appliedVolts;
    }

    public void setNutralMode(boolean isBrake) {
        
    }
     
    public void setVoltage(double volt) {
        appliedVolts = volt;
        motor.setInputVoltage(volt);
    }
     
    public void updatePeriodic() {
        motor.update(RobotConstants.kDELTA_TIME);    
        
        motorTempLog.update(getMotorTemp());
        appliedVoltsLog.update(getAppliedVolts());
        velocityLog.update(getVelocity());
        currentDrawLog.update(getCurrentDraw());
    }

}
