// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.PoseEstimation;


import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Subsystem.Swerve.SwerveSubsystem;

/** Add your docs here. */
public class PoseEstimatorConstants {

    public final static Vector<N3> ODOMETRY_DEVS = VecBuilder.fill(0.9, 0.9, 0.9);
    public final static Vector<N3> VISION_DEVS = VecBuilder.fill(0.9, 0.9, 0.9);
    
    public final static double MAX_LINEAR_VELOCITY_FOR_UPDATE = 0;//Meters per second
    public final static double MAX_ANGULAR_VELOCITY_FOR_UPDATE = 0;//Radians per second

    public static boolean VISION_UPDATE_CONSTRAINS = SwerveSubsystem.getInstance().getRobotRelativeSpeeds().vxMetersPerSecond < MAX_LINEAR_VELOCITY_FOR_UPDATE 
    && SwerveSubsystem.getInstance().getRobotRelativeSpeeds().vyMetersPerSecond < MAX_LINEAR_VELOCITY_FOR_UPDATE
    && SwerveSubsystem.getInstance().getRobotRelativeSpeeds().omegaRadiansPerSecond < MAX_ANGULAR_VELOCITY_FOR_UPDATE; 

    public static boolean ODOMETRY_UPDATE_CONSTRAINS = DriverStation.isEnabled() && !DriverStation.isTest();
}
