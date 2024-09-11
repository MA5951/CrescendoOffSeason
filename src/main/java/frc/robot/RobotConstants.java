// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ma5951.utils.StateControl.StatesTypes.State;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotControl.SuperStructure;
import frc.robot.Subsystem.PoseEstimation.VisionConstants;
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
    public static final ShootingParameters LOW_FEEDING_SHOOTING_PARAMETERS = new ShootingParameters(4000, 4000, 12 , 0);
    public static final ShootingParameters SUBWOOF_SHOOTING_PARAMETERS = new ShootingParameters(2500, 4000, 61, 0);
    public static final ShootingParameters PODIUM_SHOOTING_PARAMETERS = new ShootingParameters(4000, 8000,  36.4 , 3.5);
//    public static final ShootingParameters PODIUM_SHOOTING_PARAMETERS = new ShootingParameters(3000, 6000, 36 , 0);


    //Interpolation\[]
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
    public static final double DISTANCE_TO_CLOSE_ARM = 0.2;//

    public static final double[][] BUMPER_TO_SUBWOOFER_DISTANCE = {
       {0  , 0.25 ,   0.5 ,  0.75 ,    1 , 1.25 , 1.5  , 1.75 ,    2 , 2.30 ,  2.5 , 2.75 , 3 , 3.25 , 3.5  , 3.75 ,  4   , 4.25 , 4.5 , 4.75 },
       {64 , 57.6 , 54.96 , 47.17 , 45.1 , 41.6 , 36.57,36.4, 33.2, 30.2, 27.3, 28.5 , 26.7, 25.7, 24.7, 24.4   , 24.1, 23.9, 23.7, 23.7} 
    };                                                                           

    public static final double[][] shootingPoses = {
        {VisionConstants.CAMERA_TO_BUPMER + VisionConstants.SPEAKER_TO_SUBWOOFER + BUMPER_TO_SUBWOOFER_DISTANCE[0][0], BUMPER_TO_SUBWOOFER_DISTANCE[1][0], 0},
        {VisionConstants.CAMERA_TO_BUPMER + VisionConstants.SPEAKER_TO_SUBWOOFER + BUMPER_TO_SUBWOOFER_DISTANCE[0][1], BUMPER_TO_SUBWOOFER_DISTANCE[1][1], 0},
        {VisionConstants.CAMERA_TO_BUPMER + VisionConstants.SPEAKER_TO_SUBWOOFER + BUMPER_TO_SUBWOOFER_DISTANCE[0][2], BUMPER_TO_SUBWOOFER_DISTANCE[1][2], 0},
        {VisionConstants.CAMERA_TO_BUPMER + VisionConstants.SPEAKER_TO_SUBWOOFER + BUMPER_TO_SUBWOOFER_DISTANCE[0][3], BUMPER_TO_SUBWOOFER_DISTANCE[1][3], 0},
        {VisionConstants.CAMERA_TO_BUPMER + VisionConstants.SPEAKER_TO_SUBWOOFER + BUMPER_TO_SUBWOOFER_DISTANCE[0][4], BUMPER_TO_SUBWOOFER_DISTANCE[1][4], 0},
        {VisionConstants.CAMERA_TO_BUPMER + VisionConstants.SPEAKER_TO_SUBWOOFER + BUMPER_TO_SUBWOOFER_DISTANCE[0][5], BUMPER_TO_SUBWOOFER_DISTANCE[1][5], 0},
        {VisionConstants.CAMERA_TO_BUPMER + VisionConstants.SPEAKER_TO_SUBWOOFER + BUMPER_TO_SUBWOOFER_DISTANCE[0][6], BUMPER_TO_SUBWOOFER_DISTANCE[1][6], 0},
        {VisionConstants.CAMERA_TO_BUPMER + VisionConstants.SPEAKER_TO_SUBWOOFER + BUMPER_TO_SUBWOOFER_DISTANCE[0][7], BUMPER_TO_SUBWOOFER_DISTANCE[1][7], 0},
        {VisionConstants.CAMERA_TO_BUPMER + VisionConstants.SPEAKER_TO_SUBWOOFER + BUMPER_TO_SUBWOOFER_DISTANCE[0][8], BUMPER_TO_SUBWOOFER_DISTANCE[1][8], 0},
        {VisionConstants.CAMERA_TO_BUPMER + VisionConstants.SPEAKER_TO_SUBWOOFER + BUMPER_TO_SUBWOOFER_DISTANCE[0][9], BUMPER_TO_SUBWOOFER_DISTANCE[1][9], 0},
        {VisionConstants.CAMERA_TO_BUPMER + VisionConstants.SPEAKER_TO_SUBWOOFER + BUMPER_TO_SUBWOOFER_DISTANCE[0][10], BUMPER_TO_SUBWOOFER_DISTANCE[1][10], 0},
        {VisionConstants.CAMERA_TO_BUPMER + VisionConstants.SPEAKER_TO_SUBWOOFER + BUMPER_TO_SUBWOOFER_DISTANCE[0][11], BUMPER_TO_SUBWOOFER_DISTANCE[1][11], 0},
        {VisionConstants.CAMERA_TO_BUPMER + VisionConstants.SPEAKER_TO_SUBWOOFER + BUMPER_TO_SUBWOOFER_DISTANCE[0][12], BUMPER_TO_SUBWOOFER_DISTANCE[1][12], 0},
        {VisionConstants.CAMERA_TO_BUPMER + VisionConstants.SPEAKER_TO_SUBWOOFER + BUMPER_TO_SUBWOOFER_DISTANCE[0][13], BUMPER_TO_SUBWOOFER_DISTANCE[1][13], 0},
        {VisionConstants.CAMERA_TO_BUPMER + VisionConstants.SPEAKER_TO_SUBWOOFER + BUMPER_TO_SUBWOOFER_DISTANCE[0][14], BUMPER_TO_SUBWOOFER_DISTANCE[1][14], 0},
        {VisionConstants.CAMERA_TO_BUPMER + VisionConstants.SPEAKER_TO_SUBWOOFER + BUMPER_TO_SUBWOOFER_DISTANCE[0][15], BUMPER_TO_SUBWOOFER_DISTANCE[1][15], 0},
        {VisionConstants.CAMERA_TO_BUPMER + VisionConstants.SPEAKER_TO_SUBWOOFER + BUMPER_TO_SUBWOOFER_DISTANCE[0][16], BUMPER_TO_SUBWOOFER_DISTANCE[1][16], 0},
        {VisionConstants.CAMERA_TO_BUPMER + VisionConstants.SPEAKER_TO_SUBWOOFER + BUMPER_TO_SUBWOOFER_DISTANCE[0][17], BUMPER_TO_SUBWOOFER_DISTANCE[1][17], 0},
        {VisionConstants.CAMERA_TO_BUPMER + VisionConstants.SPEAKER_TO_SUBWOOFER + BUMPER_TO_SUBWOOFER_DISTANCE[0][18], BUMPER_TO_SUBWOOFER_DISTANCE[1][18], 0},
        {VisionConstants.CAMERA_TO_BUPMER + VisionConstants.SPEAKER_TO_SUBWOOFER + BUMPER_TO_SUBWOOFER_DISTANCE[0][19], BUMPER_TO_SUBWOOFER_DISTANCE[1][19], 0}

    };
}
