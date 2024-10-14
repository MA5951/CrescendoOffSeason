// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Arm;

import com.ma5951.utils.StateControl.StatesTypes.State;
import com.ma5951.utils.StateControl.StatesTypes.StatesConstants;
import com.ma5951.utils.Utils.ConvUtil;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.Robot;
import frc.robot.Subsystem.Arm.IOs.ArmIO;
import frc.robot.Subsystem.Arm.IOs.ArmIOReal;
import frc.robot.Subsystem.Arm.IOs.ArmIOSim;

/** Add your docs here. */
public class ArmConstants {
    public static final double INTAKE_POSE = 11.93;
    public static final double ZERO_POSE = INTAKE_POSE;
    public static final double AMP_POSE = 138;//Comp - 131//Practic - 138
    public static final double SOURCE_INTAKE_POSE = 70;

    public static final double ARM_LENGTH = 0.35;
    public static final double ARM_WEIGHT = 8.5;
    
    public static final double MG = ARM_WEIGHT * 9.8;

    public static final double kSTALL_TOURQE = 7;
    public static final int CONTROL_SLOT = 0;
    public static final double kP = 200;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kCRUSIE_VELOCITY = ConvUtil.RPMtoRPS(35);
    public static final double kACCELERATION = 270;
    public static final double kJERK = 0;
    public static final double kTOLORANCE = 0.5;//0.5
    public static final double GEAR = 1/ 0.0073;

    public static final double kARM_MOVING_THRSHOLD_RPM = 2;

    public static final double SIM_kP = 0.1;
    public static final double SIM_kD = 0;
    public static final double SIM_kI = 0;

    public static final double ACTIVE_HOME_LIMIT_ANGLE = 15;
    public static final double HOME_VOLTAGE = -0.5;

    public static final double CAN_MOVE_CURRENT_LIMIT = 25;

    public static final double INTAKE_HOLD_VALUE = -0.1;

    public static double LOWER_LIMIT = 10;
    public static double UPPER_LIMIT = 180;

    public static final double MANUEL_VOLTAGE_LIMIT = 3;


    public static final Pose3d SIM_ARM_OFFSET = new Pose3d(0.035, -0.002 ,  0.613, new Rotation3d(0, 0, 0));

    public static final double PeakCurrentLimit = 40; 
    public static final double ContinuesCurrentLimit = 30; 
    public static final double PeakCurrentTime = 0.1; 
    public static final boolean IsCurrentLimitEnabled = true; 

    public static final State IDLE = StatesConstants.IDLE;
    public static final State FOLLOW_SPEAKER = new State("FOLLOW_SPEAKER");
    public static final State PRESET_SHOOTING = new State("PRESET_SHOOTING");
    public static final State SOURCE_INTAKE = new State("SOURCE_INTAKE");
    public static final State FEEDING = new State("FEEDING");
    public static final State AMP = new State("AMP");
    public static final State INTAKE = new State("INTAKE");
    public static final State HOME = new State("HOME");

    public static final State[] SUBSYSTEM_STATES = {IDLE, FOLLOW_SPEAKER, SOURCE_INTAKE, AMP, INTAKE, HOME, FEEDING
    , PRESET_SHOOTING};

    public static final ArmIO getArmIO() {
        if (Robot.isReal()) {
            return new ArmIOReal();
        } else { 
            return new ArmIOSim();
        }
    }
}
