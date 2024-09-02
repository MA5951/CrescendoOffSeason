// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotControl;

import com.ma5951.utils.Utils.GeomUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotConstants;
import frc.robot.Subsystem.Feeder.Feeder;
import frc.robot.Subsystem.PoseEstimation.PoseEstimator;
import frc.robot.Subsystem.Shooter.Shooter;
import frc.robot.Subsystem.Swerve.SwerveSubsystem;
import frc.robot.Utils.ShootingParameters;

/** Add your docs here. */
public class SuperStructure {
    private static SuperStructure superStructure;


    private static ShootingParameters presetParameters;
    private static ShootingParameters point;
    private static Pose2d speakerPose = DriverStation.getAlliance().get() == Alliance.Red ? 
      RobotConstants.RED_SPEAKER : RobotConstants.BLUE_SPEAKER;
    private static Pose2d feedingPose = DriverStation.getAlliance().get() == Alliance.Red ? 
      new Pose2d(speakerPose.getX() - RobotConstants.FeedingOffsetY , speakerPose.getY() , speakerPose.getRotation()) : new Pose2d(speakerPose.getX() + RobotConstants.FeedingOffsetY , speakerPose.getY() , speakerPose.getRotation());
    private static InterpolatingDoubleTreeMap leftShooterInterpolation = new InterpolatingDoubleTreeMap();
    private static InterpolatingDoubleTreeMap rightShooterInterpolation = new InterpolatingDoubleTreeMap();
    private static InterpolatingDoubleTreeMap angleInterpolation = new InterpolatingDoubleTreeMap();
        
    private SuperStructure() {
        setupInterpolation();
    }


    public void setupInterpolation() {
        // for (int i = 0; i < RobotConstants.PointsArry.length; i++) {
        //     point = RobotConstants.PointsArry[i];
        //     leftShooterInterpolation.put(point.getLeftSpeed(), point.getDistance());
        //     rightShooterInterpolation.put(point.getRightSpeed(), point.getDistance());
        //     angleInterpolation.put(point.getArmAngle(), point.getDistance());
        // }
    }

    public void setPRESETParameters(ShootingParameters parameters) {
        presetParameters = parameters;
    }

    public ShootingParameters getWarmingParameters() {
        return RobotConstants.WARM_SHOOTING_PARAMETERS;
    }

    public ShootingParameters getPRESETParameters() {
        return presetParameters;
    }

    public ShootingParameters getShootingPrameters() {
        return new ShootingParameters(
            leftShooterInterpolation.get(getDistanceToSpeaker()),
            rightShooterInterpolation.get(getDistanceToSpeaker()),
            angleInterpolation.get(getDistanceToSpeaker()),
             getDistanceToSpeaker());
    }

    public ShootingParameters getFeedingPrameters() {
        return RobotConstants.FEEDING_SHOOTING_PARAMETERS;
    }

    //Cirecle between 0 to 360 //Always positive
    public double getRobotHeading() {
        return Math.abs((SwerveSubsystem.getInstance().getFusedHeading() - SwerveSubsystem.getInstance().getOffsetAngle()) % 360);
    }
 
    public boolean isInWarmUpZone() {
        return getDistanceToSpeaker() < RobotConstants.DISTANCE_TO_WARM;
    }

    public boolean isHeading(Pose2d point, double tolerance) {
        double angle = GeomUtil.getAngleBetween(point, PoseEstimator.getInstance().getEstimatedRobotPose());
        return Math.abs(getRobotHeading() - angle) < tolerance;
    }

    public boolean isHeadingForFeeding() {
        return isHeading(feedingPose , RobotConstants.FeedingTolerance);
    }

    public boolean isHeadingForShooting() {
        return isHeading(speakerPose , RobotConstants.ShootingTolerance);
    }

    public boolean isRobotMoving() {
        return SwerveSubsystem.getInstance().getRobotRelativeSpeeds() == new ChassisSpeeds(0,0,0);
    }

    public double getDistanceToSpeaker() {
        if (DriverStation.getAlliance().get() == Alliance.Blue) {
            return GeomUtil.distanceTo(RobotConstants.BLUE_AMP, PoseEstimator.getInstance().getEstimatedRobotPose());
        } else {
            return GeomUtil.distanceTo(RobotConstants.RED_SPEAKER, PoseEstimator.getInstance().getEstimatedRobotPose());
        }
    }

    public boolean isNote() {
        return Feeder.getInstance().isNoteInFeeder() || Shooter.getInstance().isNoteInShooter();
    }

    public boolean isNoteInFeeder() {
        return Feeder.getInstance().isNoteInFeeder();
    }

    public boolean isNoteInShooter() {
        return Shooter.getInstance().isNoteInShooter();
    }

    public static SuperStructure getInstance() {
        if (superStructure == null) {
            superStructure = new SuperStructure();
        }
        return superStructure;
    }
    
}
