// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.PoseEstimation;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import frc.robot.Subsystem.Swerve.SwerveConstants;

/** Add your docs here. */
public class RobotPoseEstimator {

    private PoseEstimator  poseEstimator;

    public RobotPoseEstimator() {
        poseEstimator = new PoseEstimator<SwerveDrivePoseEstimator> (); //Vision dev in meters and radians
    }
 
    public void updateVision() {

    }

}
 