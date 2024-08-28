// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ma5951.utils.StateControl.StatesTypes.State;

import frc.robot.Utils.ShootingParameters;

public class RobotConstants {

    public static final double kDELTA_TIME = 0.02;

    public static final State IDLE = new State("IDLE"); //Systems IDLE
    public static final State INTAKE = new State("INTAKE"); //Intaking a note
    public static final State EJECT = new State("EJECT"); //Ejecting a note from shooter
    public static final State WARMING = new State("WARMING"); //Warimng up shooter and arm for shooting
    public static final State AMP = new State("AMP"); //Amping
    public static final State FEEDING = new State("FEEDING"); //Feeding
    public static final State SOURCE_INTAKE = new State("SOURCE_INTAKE"); //Source intake
    public static final State STATIONARY_SHOOTING = new State("STATIONARY_SHOOTING"); //Stationary auto shoot
    public static final State PRESET_SHOOTING = new State("PRESET_SHOOTING"); //Preset shooting


    public static final double AngleFotShooting = 0;
    public static final double ShootingTolerance = 0;

    public static final double AngleForFeeding = 0;
    public static final double FeedingTolerance = 0;

    public static final ShootingParameters WARM_SHOOTING_PARAMETERS = new ShootingParameters(0, 0, 0);
}
