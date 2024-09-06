// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Shooter;

import java.util.function.Supplier;

import com.ma5951.utils.StateControl.StatesTypes.State;
import com.ma5951.utils.StateControl.StatesTypes.StatesConstants;

import frc.robot.Robot;
import frc.robot.Subsystem.Shooter.IOs.ShooterIO;
import frc.robot.Subsystem.Shooter.IOs.ShooterIOReal;
import frc.robot.Subsystem.Shooter.IOs.ShooterIOSim;

public class ShooterConstants {

    public static final int CONTROL_SLOT = 0;
    public static final double kP = 0.4;
    public static final double kI = 0.0002;
    public static final double kD = 0;
    public static final double kTOLORANCE = 50;
    public static final double GEAR = 0.5;
    public static final Supplier<Double> kTOLORANCE_BETWEEN_SIDES = () ->
        Math.abs(Shooter.getInstance().getRightSetPoint() - Shooter.getInstance().getLeftSetPoint()) + 100;

    public static final double MAX_SYSTEM_RPM = 6000 * (1 / GEAR);

    public static final double SIM_kP = 0.1;
    public static final double SIM_kD = 0;
    public static final double SIM_kI = 0;

    public static final double EJECTING_VOLTGE = -4;
    public static final double SOURCE_INTAKE_VOLTAGE = -4;

    public static final double PeakCurrentLimit = 40; 
    public static final double ContinuesCurrentLimit = 25; 
    public static final double PeakCurrentTime = 0.1; 
    public static final boolean IsCurrentLimitEnabled = true; 

    public static final State IDLE = StatesConstants.IDLE;
    public static final State WARM = new State("WARM");
    public static final State SHOOTING = new State("SHOOTING");
    public static final State FEEDING = new State("FEEDING");
    public static final State EJECTING = new State("EJECTING");
    public static final State SOURCE_INTAKE = new State("SOURCE_INTAKE");
    public static final State PRESET_SHOOTING = new State("PRESET_SHOOTING");

    public static final State[] SYSTEM_STATES = {IDLE, WARM, SHOOTING, FEEDING, EJECTING, SOURCE_INTAKE , PRESET_SHOOTING};

    public static final ShooterIO getShooterIO() {
        if (Robot.isReal()) {
            return new ShooterIOReal();
        } else {
            return new ShooterIOSim();
        }
    }
}
