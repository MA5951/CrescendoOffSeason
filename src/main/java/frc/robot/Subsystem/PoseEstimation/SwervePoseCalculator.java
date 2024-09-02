// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.PoseEstimation;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Subsystem.Swerve.CollisionDtector;
import frc.robot.Subsystem.Swerve.SkidDetector;
import frc.robot.Subsystem.Swerve.SwerveConstants;
import frc.robot.Subsystem.Swerve.SwerveOdometry;

/** Add your docs here. */
public class SwervePoseCalculator {

    private static SwervePoseCalculator swervePoseCalculator;

    //private CollisionDtector collisionDtector;
    //private SkidDetector skidDetector;
    private SwerveOdometry odometry;

    public SwervePoseCalculator() {
        //collisionDtector = new CollisionDtector(SwerveConstants.getGyro());
        //skidDetector = new SkidDetector(SwerveConstants.kinematics , () -> SwerveSubsystem.getInstance().getSwerveModuleStates());
        odometry = new SwerveOdometry();
    }

    
    public void update() {
        if (PoseEstimatorConstants.ODOMETRY_UPDATE_CONSTRAINS) {
            odometry.update();
        }
        //collisionDtector.update();
        //skidDetector.update();
    }

    public Pose2d getEstimatesPose() {
        return odometry.getPose();
    }

    public static SwervePoseCalculator getInstance() {
        if (swervePoseCalculator == null) {
          swervePoseCalculator = new SwervePoseCalculator();
        }
        return swervePoseCalculator;
        }



}