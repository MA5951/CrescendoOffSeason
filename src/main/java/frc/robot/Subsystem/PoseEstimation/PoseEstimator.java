// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.PoseEstimation;


import com.ma5951.utils.Logger.LoggedBool;
import com.ma5951.utils.Logger.LoggedPose2d;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Subsystem.Swerve.SwerveConstants;
import frc.robot.Subsystem.Swerve.SwerveSubsystem;

/** Add your docs here. */
public class PoseEstimator {
    private static PoseEstimator poseEstimator;

    private SwerveDrivePoseEstimator robotPoseEstimator;
    private Vision vision;
    private SwerveSubsystem swerve = SwerveSubsystem.getInstance();
    private Pose2d lastOdometryPose = new Pose2d( 0 , 0 , new Rotation2d(0));
    private Pose2d lastVisionPose = new Pose2d();
    private boolean firstUpdate = true;

    private LoggedPose2d estimatedRobotPose;
    private LoggedBool odometryUpdateConstrains;
    private LoggedBool visionUpdateConstrains;


    public PoseEstimator() {
        vision = Vision.getInstance();
        robotPoseEstimator = new SwerveDrivePoseEstimator(SwerveConstants.kinematics , swerve.getRotation2d() , 
        swerve.getSwerveModulePositions() ,
        new Pose2d(),
        PoseEstimatorConstants.ODOMETRY_DEVS,//Oodmetry Devs
        PoseEstimatorConstants.VISION_DEVS);//Vision Devs

        estimatedRobotPose = new LoggedPose2d("/Pose Estimator/Estimated Robot Pose");
        odometryUpdateConstrains = new LoggedBool("/Pose Estimator/Odometry Update Constrains");
        visionUpdateConstrains = new LoggedBool("/Pose Estimator/Vision Update Constrains");
    }

    public void resetPose(Pose2d pose) {
        robotPoseEstimator.resetPosition(swerve.getRotation2d() , swerve.getSwerveModulePositions() ,pose) ;
    }
    
    public void updateOdometry() {
        robotPoseEstimator.update(swerve.getRotation2d(), swerve.getSwerveModulePositions());
    }

    // ((Math.abs(vision.getEstiman().getTranslation().getDistance(lastVisionPose.getTranslation()) -
    //             getEstimatedRobotPose().getTranslation().getDistance(lastOdometryPose.getTranslation()))
    //              <= PoseEstimatorConstants.VISION_TO_ODOMETRY_DIFRANCE ) ||  firstUpdate)

    public void updateVision() {
        if (PoseEstimatorConstants.VISION_UPDATE_CONSTRAINS.get()) {
            if (vision.getEstiman() != new Pose2d()  && (Math.abs(vision.getEstiman().getTranslation().getDistance(getEstimatedRobotPose().getTranslation())) <= PoseEstimatorConstants.VISION_TO_ODOMETRY_DIFRANCE
            || true)
           ) {
                
                lastOdometryPose = getEstimatedRobotPose();
                lastVisionPose = vision.getEstiman();
                firstUpdate = false;
                //robotPoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(getXYVisionDeviation(), getXYVisionDeviation(), 9999999));
                robotPoseEstimator.addVisionMeasurement(vision.getEstiman(), vision.getTimeStamp());
            }
        }
    }

    public Pose2d getEstimatedRobotPose() {
        return robotPoseEstimator.getEstimatedPosition();
    }

    public double getXYVisionDeviation() {
        return PoseEstimatorConstants.xyDEVS_COEFFICIENT * Math.pow(vision.getDistance(), 2) / vision.getTagArea();
    }

    public void update() {
        updateOdometry();
        updateVision();
        estimatedRobotPose.update(getEstimatedRobotPose());
        odometryUpdateConstrains.update(PoseEstimatorConstants.ODOMETRY_UPDATE_CONSTRAINS.get());
        visionUpdateConstrains.update(PoseEstimatorConstants.VISION_UPDATE_CONSTRAINS.get());

    }


    public static PoseEstimator getInstance() {
        if (poseEstimator == null) {
          poseEstimator = new PoseEstimator();
        }
        return poseEstimator;
        }



}
