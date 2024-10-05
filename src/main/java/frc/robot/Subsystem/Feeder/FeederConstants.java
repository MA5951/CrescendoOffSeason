// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Feeder;

import com.ma5951.utils.StateControl.StatesTypes.State;
import com.ma5951.utils.StateControl.StatesTypes.StatesConstants;

import frc.robot.Robot;
import frc.robot.Subsystem.Feeder.IOs.FeederIO;
import frc.robot.Subsystem.Feeder.IOs.FeederIOReal;
import frc.robot.Subsystem.Feeder.IOs.FeederIOSim;


/** Add your docs here. */
public class FeederConstants {

    public static final double FORWARD_VOLTAGE = 3.5; 
    public static final double REVERS_VOLTAG = -6; 

    public static final double FORWARD_ADJUST_VOLTAG = 2; 
    public static final double REVERS_ADJUST_VOLTAG = -2; 

    public static final double GEAR = 2.66; 

    public static final double PeakCurrentLimit = 30; 
    public static final double ContinuesCurrentLimit = 25; 
    public static final double PeakCurrentTime = 0.1; 
    public static final boolean IsCurrentLimitEnabled = true; 

    public static final double SHOOTING_CONDITION_DEBOUNCER = 0.1;

    public static final State IDLE = StatesConstants.IDLE;
    public static final State FORWARD = new State("FORWARD");
    public static final State AMP_REALES = new State("AMP_REALES");
    public static final State REVERSE = new State("REVERSE");
    public static final State NOTE_ADJUSTING = new State("NOTE_ADJUSTING");

    public static final State[] SYSTEM_STATES = {IDLE, FORWARD, REVERSE, AMP_REALES , NOTE_ADJUSTING};

    public static final FeederIO getFeederIO() {
        if (Robot.isReal()) {
            return new FeederIOReal();
        } else {
            return new FeederIOSim();
        }
    }
}
