// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Swerve;



import com.ma5951.utils.Utils.DriverStationUtil;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.Subsystem.PoseEstimation.PoseEstimator;

/** Add your docs here. */
public class SwerveAutoFollower {

    private SwerveSubsystem swerve = SwerveSubsystem.getInstance();
    private PoseEstimator poseEstimate = PoseEstimator.getInstance();
    private RobotConfig config;

    public SwerveAutoFollower() {
        try{
            config = RobotConfig.fromGUISettings();
          } catch (Exception e) {
              e.printStackTrace();
          }
          
        AutoBuilder.configure(
            () -> poseEstimate.getEstimatedRobotPose(), 
            pose -> poseEstimate.resetPose(pose),
            () -> swerve.getRobotRelativeSpeeds(),
            speeds -> swerve.drive(speeds, true),
            new PPHolonomicDriveController(
                new PIDConstants(0, 0, 0),
                new PIDConstants(0, 0, 0)), 
            config,
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                  return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
              }, swerve);
        
        
    }

    public static Command buildAuto(String autoName) {
        return new PathPlannerAuto(autoName);
    }
}
