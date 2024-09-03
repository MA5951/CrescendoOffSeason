// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ma5951.utils.StateControl.StatesTypes.State;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotControl.SuperStructure;
import frc.robot.Utils.ShootingParameters;

public class RobotConstants {

    public static final double kDELTA_TIME = 0.02;

    public static final SuperStructure SUPER_STRUCTURE = new SuperStructure();

    public static final State IDLE = new State("IDLE"); //Systems IDLE 
    public static final State INTAKE = new State("INTAKE"); //Intaking a note //Did
    public static final State EJECT = new State("EJECT"); //Ejecting a note from shooter //Did
    public static final State WARMING = new State("WARMING"); //Warimng up shooter and arm for shooting //Did
    public static final State AMP = new State("AMP"); //Amping //Did
    public static final State FEEDING = new State("FEEDING"); //Feeding 
    public static final State SOURCE_INTAKE = new State("SOURCE_INTAKE"); //Source intake
    public static final State STATIONARY_SHOOTING = new State("STATIONARY_SHOOTING"); //Stationary auto shoot //Did
    public static final State PRESET_SHOOTING = new State("PRESET_SHOOTING"); //Preset shooting

    
    public static final double ShootingTolerance = 15;

    public static final double FeedingOffsetY = 3;
    public static final double FeedingOffsetX = 1;
    public static final double FeedingTolerance = 15;

    public static final ShootingParameters WARM_SHOOTING_PARAMETERS = new ShootingParameters(0, 0, 0 , 0);

    public static final ShootingParameters FEEDING_SHOOTING_PARAMETERS = new ShootingParameters(0, 0, 0 , 0);
    public static final ShootingParameters LOW_FEEDING_SHOOTING_PARAMETERS = new ShootingParameters(0, 0, 0 , 0);
    public static final ShootingParameters SUBWOOF_SHOOTING_PARAMETERS = new ShootingParameters(1, 1, 50 , 0);
    public static final ShootingParameters PODIUM_SHOOTING_PARAMETERS = new ShootingParameters(1, 1, 20 , 0);


    //Interpolation
    //Shooting Points
    private static final ShootingParameters Point1 = new ShootingParameters(0, 0, 0 , 0);
    private static final ShootingParameters Point2 = new ShootingParameters(0, 0, 0 , 0);
    private static final ShootingParameters Point3 = new ShootingParameters(0, 0, 0 , 0);
    private static final ShootingParameters Point4 = new ShootingParameters(0, 0, 0 , 0);
    private static final ShootingParameters Point5 = new ShootingParameters(0, 0, 0 , 0);
    private static final ShootingParameters Point6 = new ShootingParameters(0, 0, 0 , 0);

    

    public static final ShootingParameters[] PointsArry = new ShootingParameters[] {
        Point1,
        Point2,
        Point3,
        Point4,
        Point5,
        Point6
    };

    //XY = 0 is the corenr of the red humean source
    //Y is side to side form driver station view
    //X is front to back from driver station view 
    //FieldConstants
    public static final Pose2d BLUE_SPEAKER = new Pose2d(0 , 5.548 , new Rotation2d(0));
    public static final Pose2d RED_SPEAKER = new Pose2d(16.579 , 5.548 , new Rotation2d(0));
    public static final Pose2d BLUE_AMP = new Pose2d(1.842 , 8.204 , new Rotation2d(0));
    public static final Pose2d RED_AMP = new Pose2d(14.701 , 8.204 , new Rotation2d(0));

    public static final double DISTANCE_TO_WARM = 5;//Warm raidus in meters
}
