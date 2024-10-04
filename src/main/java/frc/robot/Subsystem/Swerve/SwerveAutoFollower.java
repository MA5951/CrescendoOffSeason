// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Swerve;



import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;

import frc.robot.Subsystem.PoseEstimation.PoseEstimator;

/** Add your docs here. */
public class SwerveAutoFollower {

    private SwerveSubsystem swerve = SwerveSubsystem.getInstance();
    private PoseEstimator poseEstimate = PoseEstimator.getInstance();

    public SwerveAutoFollower() {
        AutoBuilder.configureHolonomic(
            poseEstimate.getEstimatedRobotPose(), 
            poseEstimate.resetPose(null), 
            swerve.getRobotRelativeSpeeds(), 
            swerve.generateStates(null, true), 
            new HolonomicPathFollowerConfig(null, null, 0, 0, null), null, swerve);
    }
}
