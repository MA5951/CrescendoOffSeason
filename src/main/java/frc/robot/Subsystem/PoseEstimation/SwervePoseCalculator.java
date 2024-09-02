// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.PoseEstimation;


import frc.robot.Subsystem.Swerve.SwerveOdometry;

/** Add your docs here. */
public class SwervePoseCalculator {

    private static SwervePoseCalculator swervePoseCalculator;

    private SwerveOdometry odometry;

    public SwervePoseCalculator() {
        odometry = new SwerveOdometry();
    }

    
    public void update() {
        odometry.update();
    }

    public SwerveOdometry getOdometry() {
        return odometry;
    }

    public static SwervePoseCalculator getInstance() {
        if (swervePoseCalculator == null) {
          swervePoseCalculator = new SwervePoseCalculator();
        }
        return swervePoseCalculator;
        }



}
