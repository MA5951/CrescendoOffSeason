// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Feeder;

import com.ma5951.utils.StateControl.StatesTypes.State;

import frc.robot.Robot;
import frc.robot.Subsystem.Feeder.IOs.FeederIO;
import frc.robot.Subsystem.Feeder.IOs.FeederIOReal;
import frc.robot.Subsystem.Feeder.IOs.FeederIOSim;


/** Add your docs here. */
public class FeederConstants {

    public static final double FEEDER_POWER = 0; //TODO
    public static final double EJECT_POWER = 0; //TODO

    public static final double GEAR = 2.66; //TODO

    public static final State IDLE = new State("IDLE");
    public static final State FEEDING = new State("FEEDING");
    public static final State EJECTING = new State("EJECTING");


    public static final FeederIO getFeederIO() {
        if (Robot.isReal()) {
            return new FeederIOReal();
        } else {
            return new FeederIOSim();
        }
    }
}
