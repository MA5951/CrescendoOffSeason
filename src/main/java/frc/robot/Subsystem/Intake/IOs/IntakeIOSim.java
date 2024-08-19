// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Intake.IOs;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Subsystem.Intake.IntakeConstants;

/** Add your docs here. */
public class IntakeIOSim implements IntakeIO{

    private DCMotorSim motor;
    private double appliedVolts;

    public IntakeIOSim() {
        motor = new DCMotorSim(DCMotor.getKrakenX60(1), IntakeConstants.Gear, 0);
     
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
       motor.update(0);  
    }





    



}
