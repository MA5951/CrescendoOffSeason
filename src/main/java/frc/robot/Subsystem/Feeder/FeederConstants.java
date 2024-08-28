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

    public static final double FEEDER_POWER = 0; //TODO
    public static final double EJECT_POWER = 0; //TODO

    public static final double GEAR = 2.66; //TODO

    public static final State IDLE = StatesConstants.IDLE;
    public static final State FORWARD = new State("FORWARD");
    public static final State AMP_REALES = new State("AMP_REALES");
    public static final State REVERSE = new State("REVERSE");
    public static final State NOTE_ADJUSTING = new State("NOTE_ADJUSTING");

    public static final State[] SYSTEM_STATES = {IDLE, FORWARD, REVERSE, AMP_REALES};

    public static final FeederIO getFeederIO() {
        if (Robot.isReal()) {
            return new FeederIOReal();
        } else {
            return new FeederIOSim();
        }
    }
}
