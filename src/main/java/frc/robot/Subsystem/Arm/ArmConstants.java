// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Arm;

import com.ma5951.utils.StateControl.StatesTypes.State;

import frc.robot.Robot;
import frc.robot.Subsystem.Arm.IOs.ArmIO;
import frc.robot.Subsystem.Arm.IOs.ArmIOReal;
import frc.robot.Subsystem.Arm.IOs.ArmIOSim;

/** Add your docs here. */
public class ArmConstants {
    public static final double OFFSET = 0;
    public static final double INTAKE_POSE = 11.93;
    public static final double AMP_POSE = 137.32;
    public static final double SOURCE_INTAKE_POSE = 0;

    public static final double ARM_LENGTH = 0;
    public static final double ARM_WEIGHT = 0;
    
    public static final int CONTROL_SLOT = 0;
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double kA = 0;
    public static final double kV = 0;
    public static final double kCRUSIE_VELOCITY = 0;
    public static final double kACCELERATION = 0;
    public static final double kJERK = 0;
    public static final double kTOLORANCE = 1;
    public static final double GEAR = 109.89;

    public static final double SIM_kP = 1.1;
    public static final double SIM_kD = 0;
    public static final double SIM_kI = 0;

    public static final double HOME_CURRENTLIMIT = 0;
    public static final double ANGLE_LIMIT = 0;

    public static final double PeakCurrentLimit = 0; 
    public static final double ContinuesCurrentLimit = 0; 
    public static final double PeakCurrentTime = 0; 
    public static final boolean IsCurrentLimitEnabled = true; 

    public static final State IDLE = new State("IDLE");
    public static final State FOLLOW_SPEAKER = new State("FOLLOW_SPEAKER");
    public static final State SOURCE_INTAKE = new State("SOURCE_INTAKE");
    public static final State AMP = new State("AMP");
    public static final State INTAKE = new State("INTAKE");
    public static final State HOME = new State("HOME");

    public static final State[] SUBSYSTEM_STATES = {IDLE, FOLLOW_SPEAKER, SOURCE_INTAKE, AMP, INTAKE, HOME};


    public static final ArmIO getArmIO() {
        if (Robot.isReal()) {
            return new ArmIOReal();
        } else { 
            return new ArmIOSim();
        }
    }
}
