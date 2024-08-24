// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Arm;

import frc.robot.Robot;
import frc.robot.Subsystem.Arm.IOs.ArmIO;
import frc.robot.Subsystem.Arm.IOs.ArmIOReal;
import frc.robot.Subsystem.Arm.IOs.ArmIOSim;

/** Add your docs here. */
public class ArmConstants {
    public static final double OGGSET = 0;
    public static final double INTAKE_POSE = 0;
    public static final double ARM_POSE = 0;
    public static final double SUBOFFER_POSE = 0;
    public static final double PODIOM_POSE = 0;
    public static final double FEED_POSE = 0;
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
    public static final double kTOLORANCE = 0;
    public static final double GEAR = 109.89;

    public static final double SIM_kP = 0;
    public static final double SIM_kD = 0;
    public static final double SIM_kI = 0;

    public static final double HOME_CURRENTLIMIT = 0;
    public static final double ANGLE_LIMIT = 0;

    public static final double PeakCurrentLimit = 0; 
    public static final double ContinuesCurrentLimit = 0; 
    public static final double PeakCurrentTime = 0; 
    public static final boolean IsCurrentLimitEnabled = true; 

    public static final ArmIO getArmIO() {
        if (Robot.isReal()) {
            return new ArmIOReal();
        } else { 
            return new ArmIOSim();
        }
    }
}
