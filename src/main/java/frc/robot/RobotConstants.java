// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ma5951.utils.StateControl.StatesTypes.State;

public class RobotConstants {

    public static final double kDELTA_TIME = 0.02;

    public static final State IDLE = new State("IDLE");
    public static final State INTAKE = new State("INTAKE");
    public static final State EJECT = new State("EJECT");
    public static final State WARMING = new State("WARMING");
    public static final State AMP = new State("AMP");
    public static final State FEEDING = new State("FEEDING");
    public static final State SOURCE_INTAKE = new State("SOURCE_INTAKE");
    public static final State STATIONARY_SHOOTING = new State("STATIONARY_SHOOTING");
    public static final State PODIUM_SHOOTING = new State("STATIONARY_SHOOTING");
    public static final State SUBWOOPER_SHOOTING = new State("STATIONARY_SHOOTING");


}
