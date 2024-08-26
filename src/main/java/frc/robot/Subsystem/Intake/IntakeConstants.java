// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Intake;

import com.ma5951.utils.StateControl.StatesTypes.State;

import frc.robot.Robot;
import frc.robot.Subsystem.Intake.IOs.IntakeIO;
import frc.robot.Subsystem.Intake.IOs.IntakeIOReal;
import frc.robot.Subsystem.Intake.IOs.IntakeIOSim;

/** Add your docs here. */
public class IntakeConstants {

    public static final double INTAKE_POWER = 0; //TODO
    public static final double EJECT_POWER = 0; //TODO
    
    public static final double Gear = 0.333; //TODO
    
    
    public static final double PeakCurrentLimit = 0; //TODO
    public static final double ContinuesCurrentLimit = 0; //TODO
    public static final double PeakCurrentTime = 0; //TODO
    public static final boolean IsCurrentLimitEnabled = true; //TODO

    public static final State IDLE = new State("IDLE");
    public static final State INTAKING = new State("INTAKING");
    public static final State EJECTING = new State("EJECTING");

    public static final IntakeIO getIntakeIO() {
        if (Robot.isReal()) {
            return new IntakeIOReal();
        } else {
            return new IntakeIOSim();
        }
    }





}
