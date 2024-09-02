// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.PoseEstimation;


import com.ma5951.utils.Logger.LoggedBool;
import com.ma5951.utils.Logger.LoggedPose2d;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Subsystem.Swerve.SwerveConstants;
import frc.robot.Subsystem.Swerve.SwerveSubsystem;

/** Add your docs here. */
public class PoseEstimator {
    private static PoseEstimator swervePoseCalculator;

    private SwerveDrivePoseEstimator poseEstimator;
    private Vision vision = Vision.getInstance();
    private SwerveSubsystem swerve = SwerveSubsystem.getInstance();;

    private LoggedPose2d estimatedRobotPose;
    private LoggedBool odometryUpdateConstrains;
    private LoggedBool visionUpdateConstrains;


    public PoseEstimator() {
        poseEstimator = new SwerveDrivePoseEstimator(SwerveConstants.kinematics , swerve.getRotation2d() , 
        swerve.getSwerveModulePositions() ,
        new Pose2d(),
        PoseEstimatorConstants.ODOMETRY_DEVS,//Oodmetry Devs
        PoseEstimatorConstants.VISION_DEVS);//Vision Devs

        estimatedRobotPose = new LoggedPose2d("/PoseEstimator/Estimated Robot Pose");
        odometryUpdateConstrains = new LoggedBool("/PoseEstimator/Odometry Update Constrains");
        visionUpdateConstrains = new LoggedBool("/PoseEstimator/Vision Update Constrains");
        
    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(swerve.getRotation2d() , swerve.getSwerveModulePositions() ,pose) ;
    }
    
    public void updateOdometry() {
        if (PoseEstimatorConstants.ODOMETRY_UPDATE_CONSTRAINS) {
            poseEstimator.updateWithTime(Timer.getFPGATimestamp() ,swerve.getRotation2d(), swerve.getSwerveModulePositions());
        }
        
    }

    public void updateVision() {
        if (PoseEstimatorConstants.VISION_UPDATE_CONSTRAINS) {
            poseEstimator.addVisionMeasurement(vision.getEstiman(), vision.getTimeStamp());
        }
    }

    public Pose2d getEstimatedRobotPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void update() {
        updateOdometry();
        updateVision();
        estimatedRobotPose.update(getEstimatedRobotPose());
        odometryUpdateConstrains.update(PoseEstimatorConstants.ODOMETRY_UPDATE_CONSTRAINS);
        visionUpdateConstrains.update(PoseEstimatorConstants.VISION_UPDATE_CONSTRAINS);

    }


    public static PoseEstimator getInstance() {
        if (swervePoseCalculator == null) {
          swervePoseCalculator = new PoseEstimator();
        }
        return swervePoseCalculator;
        }



}
