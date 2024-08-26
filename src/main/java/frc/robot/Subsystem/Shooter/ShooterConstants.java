// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Shooter;

import frc.robot.Robot;
import frc.robot.Subsystem.Shooter.IOs.ShooterIO;
import frc.robot.Subsystem.Shooter.IOs.ShooterIOReal;
import frc.robot.Subsystem.Shooter.IOs.ShooterIOSim;

public class ShooterConstants {

    public static final int CONTROL_SLOT = 0;
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kTOLORANCE = 0;
    public static final double GEAR = 109.89;

    public static final double SIM_kP = 0.001;
    public static final double SIM_kD = 0;
    public static final double SIM_kI = 0;

    public static final double PeakCurrentLimit = 0; 
    public static final double ContinuesCurrentLimit = 0; 
    public static final double PeakCurrentTime = 0; 
    public static final boolean IsCurrentLimitEnabled = true; 


    public static final ShooterIO getShooterIO() {
        if (Robot.isReal()) {
            return new ShooterIOReal();
        } else {
            return new ShooterIOSim();
        }
    }
}
