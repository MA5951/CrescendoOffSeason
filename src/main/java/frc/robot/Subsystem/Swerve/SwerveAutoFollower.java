// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Swerve;



import com.ma5951.utils.Logger.LoggedPose2d;
import com.ma5951.utils.Utils.DriverStationUtil;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystem.PoseEstimation.PoseEstimator;

/** Add your docs here. */
public class SwerveAutoFollower {

    private SwerveSubsystem swerve = SwerveSubsystem.getInstance();
    private PoseEstimator poseEstimate = PoseEstimator.getInstance();
    private LoggedPose2d targetPoseLog;
    private LoggedPose2d currentPoseLog;

    public SwerveAutoFollower() {
        targetPoseLog = new LoggedPose2d("/Auto/Target Pose");
        currentPoseLog = new LoggedPose2d("/Auto/Current Pose");
        
        AutoBuilder.configureHolonomic(
            () -> poseEstimate.getEstimatedRobotPose(), 
            pose -> poseEstimate.resetPose(pose), 
            () -> swerve.getRobotRelativeSpeeds(), 
            speeds -> swerve.drive(speeds, true), 
            new HolonomicPathFollowerConfig(
                new PIDConstants(0.9, 0, 0), 
                new PIDConstants(0.8, 0, 0), 
                SwerveConstants.MAX_VELOCITY, 
                SwerveConstants.RADIUS, 
                new ReplanningConfig()), 
                () -> {

                var alliance = DriverStationUtil.getAlliance();
                return alliance == DriverStation.Alliance.Red;
            }, swerve);

        PathPlannerLogging.setLogTargetPoseCallback(pose -> targetPoseLog.update(pose));
        PathPlannerLogging.setLogCurrentPoseCallback(pose -> currentPoseLog.update(pose));
    }

    public static Command buildAuto(String autoName) {
        return AutoBuilder.buildAuto(autoName);
    }
}
