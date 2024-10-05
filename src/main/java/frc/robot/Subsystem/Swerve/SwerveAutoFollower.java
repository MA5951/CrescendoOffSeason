// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Swerve;



import com.ma5951.utils.Utils.DriverStationUtil;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystem.PoseEstimation.PoseEstimator;

/** Add your docs here. */
public class SwerveAutoFollower {

    private SwerveSubsystem swerve = SwerveSubsystem.getInstance();
    private PoseEstimator poseEstimate = PoseEstimator.getInstance();

    public SwerveAutoFollower() {
        AutoBuilder.configureHolonomic(
            () -> poseEstimate.getEstimatedRobotPose(), 
            pose -> poseEstimate.resetPose(pose), 
            () -> swerve.getRobotRelativeSpeeds(), 
            speeds -> swerve.drive(speeds, true), 
            new HolonomicPathFollowerConfig(
                new PIDConstants(0.05, 0, 0), 
                new PIDConstants(0.7, 0, 0), 
                SwerveConstants.MAX_VELOCITY, 
                SwerveConstants.RADIUS, 
                new ReplanningConfig()), 
                () -> {

                var alliance = DriverStationUtil.getAlliance();
                return alliance == DriverStation.Alliance.Red;
            }, swerve);
    }

    public static Command buildAuto(String autoName) {
        return AutoBuilder.buildAuto(autoName);
    }
}
