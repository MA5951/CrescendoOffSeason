// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Shooter.IOs;


public interface ShooterIO {

    boolean getBeamBraker();

    double getLeftCurrentDraw(); //Return Current Draw In Amp

    double getLeftVelocity(); //Retrun System Velocity In RPM

    double getLeftMotorTemp(); //Return Motor Temp In Celecuis

    double getLeftAppliedVolts(); //Return Applied Volts

    void  setLeftNutralMode(boolean isBrake); //Sets between coast and brake 

    void setLeftSpeedSetPoint(double setPoint , double feedforward); //Sets setPoint in RPM

    void setLeftVoltage(double volt); //Sets motor voltage between -12 to 12

    ///

    double getRightCurrentDraw(); //Return Current Draw In Amp

    double getRightVelocity(); //Retrun System Velocity In RPM

    double getRightMotorTemp(); //Return Motor Temp In Celecuis

    double getRightAppliedVolts(); //Return Applied Volts

    double getLeftError();

    double getRightError();

    void  setRightNutralMode(boolean isBrake); //Sets between coast and brake 

    void setRightSpeedSetPoint(double setPoint , double feedforward); //Sets setPoint in RPM

    void setRightVoltage(double volt); //Sets motor voltage between -12 to 12

    void updatePeriodic(); //Update Periodic

    void updatePIDValues(double Kp , double Ki , double Kd);

    void setShooterNutralMode(boolean isBrake);
}
