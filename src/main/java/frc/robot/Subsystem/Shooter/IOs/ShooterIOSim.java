// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Shooter.IOs;

import com.ma5951.utils.Logger.LoggedDouble;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.PortMap;
import frc.robot.RobotConstants;
import frc.robot.Subsystem.Shooter.ShooterConstants;

public class ShooterIOSim implements ShooterIO{

    private DCMotorSim leftMotor;
    private DCMotorSim rightMotor;
    private PIDController leftPidController = new PIDController(0.1, 0, 0);
    private PIDController rightPidController = new PIDController(0.1, 0, 0);
    private double LappliedVolts = 0;
    private double RappliedVolts = 0;

    private DigitalInput beambraker;

    private LoggedDouble LmotorTempLog;
    private LoggedDouble LappliedVoltsLog;
    private LoggedDouble LvelocityLog;
    private LoggedDouble LcurrentDrawLog;

    private LoggedDouble RmotorTempLog;
    private LoggedDouble RappliedVoltsLog;
    private LoggedDouble RvelocityLog;
    private LoggedDouble RcurrentDrawLog;

    public ShooterIOSim() {
        leftMotor = new DCMotorSim(DCMotor.getFalcon500(1), ShooterConstants.GEAR, 0.025);
        rightMotor = new DCMotorSim(DCMotor.getFalcon500(1), ShooterConstants.GEAR, 0.025);

        beambraker = new DigitalInput(PortMap.Shooter.DIO_ShooterSensor);

        LmotorTempLog = new LoggedDouble("/Subsystems/Shooter/Sim/Left Motor/Motor Temp");
        LappliedVoltsLog = new LoggedDouble("/Subsystems/Shooter/Sim/Left Motor/Motor Applied Volts");
        LvelocityLog = new LoggedDouble("/Subsystems/Shooter/Sim/Left Motor/Motor Velocity");
        LcurrentDrawLog = new LoggedDouble("/Subsystems/Shooter/Sim/Left Motor/Motor Current Draw");

        RmotorTempLog = new LoggedDouble("/Subsystems/Shooter/Sim/Right Motor/Motor Temp");
        RappliedVoltsLog = new LoggedDouble("/Subsystems/Shooter/Sim/Right Motor/Motor Applied Volts");
        RvelocityLog = new LoggedDouble("/Subsystems/Shooter/Sim/Right Motor/Motor Velocity");
        RcurrentDrawLog = new LoggedDouble("/Subsystems/Shooter/Sim/Right Motor/Motor Current Draw");
    }

    public boolean getBeamBraker() {
        return beambraker.get();
    }

    public double getLeftCurrentDraw() {
        return leftMotor.getCurrentDrawAmps();
    }

    public double getLeftVelocity() {
        return leftMotor.getAngularVelocityRPM();
    }

    public double getLeftMotorTemp() {
        return 0;
    }

    public double getLeftAppliedVolts() {
        return LappliedVolts;
    }

    public void setLeftNutralMode(boolean isBrake) {
        
    }

    public void setLeftSpeedSetPoint(double setPoint , double feedforward) {//TODO: feed forward
        setLeftVoltage(leftPidController.calculate(getLeftVelocity(), setPoint));
    }

    public void setLeftVoltage(double volt) {
        LappliedVolts = volt;
        leftMotor.setInputVoltage(volt);
    }

    public double getRightCurrentDraw() {
        return rightMotor.getCurrentDrawAmps();
    }

    public double getRightVelocity() {
        return rightMotor.getAngularVelocityRPM();
    }

    public double getRightMotorTemp() {
        return 0;
    }

    public double getRightAppliedVolts() {
        return RappliedVolts;
    }

    public void setRightNutralMode(boolean isBrake) {
        
    }

    public void setRightSpeedSetPoint(double setPoint , double feedforward) {//TODO: feed forward
        setRightVoltage(rightPidController.calculate(getRightVelocity(), setPoint));
    }

    public void setRightVoltage(double volt) {
        RappliedVolts = volt;
        rightMotor.setInputVoltage(volt);
    }

    public void setShooterNutralMode(boolean isBrake) {
        setLeftNutralMode(isBrake);
        setRightNutralMode(isBrake);
    }    

    public void updatePIDValues(double Kp, double Ki, double Kd) {
    }

    public void updatePeriodic() {
        if (DriverStation.isDisabled()) {
            setRightVoltage(0);
            setLeftVoltage(0);
        }
        
        leftMotor.update(RobotConstants.kDELTA_TIME);
        rightMotor.update(RobotConstants.kDELTA_TIME);

        LmotorTempLog.update(getLeftMotorTemp());
        LappliedVoltsLog.update(getLeftAppliedVolts());
        LvelocityLog.update(getLeftVelocity());
        LcurrentDrawLog.update(getLeftCurrentDraw());
        RmotorTempLog.update(getRightMotorTemp());
        RappliedVoltsLog.update(getRightAppliedVolts());
        RvelocityLog.update(getRightVelocity());
        RcurrentDrawLog.update(getRightCurrentDraw());
    }


}
