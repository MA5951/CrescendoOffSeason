// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotControl;

import com.ma5951.utils.Logger.LoggedBool;
import com.ma5951.utils.Logger.LoggedDouble;
import com.ma5951.utils.Utils.ConvUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.Subsystem.Feeder.Feeder;
import frc.robot.Subsystem.PoseEstimation.PoseEstimator;
import frc.robot.Subsystem.Shooter.Shooter;
import frc.robot.Subsystem.Swerve.SwerveSubsystem;
import frc.robot.Utils.ShootingParameters;

/** Add your docs here. */
public class SuperStructure {


    private ShootingParameters presetParameters;
    private ShootingParameters point;
    private Pose2d speakerPose = new Pose2d(0 , 5.548 , new Rotation2d());//TODO
    private Pose2d feedingPose = new Pose2d(speakerPose.getX() + RobotConstants.FeedingOffsetY , speakerPose.getY() + RobotConstants.FeedingOffsetX , speakerPose.getRotation());;
    private InterpolatingDoubleTreeMap leftShooterInterpolation = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap rightShooterInterpolation = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap angleInterpolation = new InterpolatingDoubleTreeMap();
    private boolean updatedAfterDS = false;

    private LoggedBool isNoteLog;
    private LoggedBool isInWarmupLog;
    private LoggedBool isHeadingForShootingLog;
    private LoggedBool isHeadingForFeedingLog;
    private LoggedBool isRobotMovingLog;
    private LoggedDouble distanceToSpeakerLog;
    private LoggedDouble robotHeadingLog;
        
    public SuperStructure() {
        setupInterpolation();
        isNoteLog = new LoggedBool("/SuperStructure/Is Note");
        isInWarmupLog = new LoggedBool("/SuperStructure/Is In Warmup");
        isHeadingForShootingLog = new LoggedBool("/SuperStructure/Is Heading For Shooting");
        isHeadingForFeedingLog = new LoggedBool("/SuperStructure/Is Heading For Feeding");
        isRobotMovingLog = new LoggedBool("/SuperStructure/Is Robot Moving");
        distanceToSpeakerLog = new LoggedDouble("/SuperStructure/Distance To Speaker");
        robotHeadingLog = new LoggedDouble("/SuperStructure/Robot Heading");
        
    }

    public void updateAfterDSConnect() {//TODO
        if (!updatedAfterDS && !DriverStation.getAlliance().isEmpty()) {
            updatedAfterDS = true;
            speakerPose = DriverStation.getAlliance().get() == Alliance.Red ? 
            RobotConstants.RED_SPEAKER : RobotConstants.BLUE_SPEAKER;
            feedingPose = DriverStation.getAlliance().get() == Alliance.Red ? 
            new Pose2d(speakerPose.getX() - RobotConstants.FeedingOffsetY , speakerPose.getY() + RobotConstants.FeedingOffsetX , speakerPose.getRotation()) :
            new Pose2d(speakerPose.getX() + RobotConstants.FeedingOffsetY , speakerPose.getY() + RobotConstants.FeedingOffsetX , speakerPose.getRotation());
        }
    }

    public void setupInterpolation() {
        for (int i = 0; i < RobotConstants.PointsArry.length; i++) {
            point = RobotConstants.PointsArry[i];
            leftShooterInterpolation.put(point.getLeftSpeed(), point.getDistance());
            rightShooterInterpolation.put(point.getRightSpeed(), point.getDistance());
            angleInterpolation.put(point.getArmAngle(), point.getDistance());
        }
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
        // return new ShootingParameters(
        //     leftShooterInterpolation.get(getDistanceToSpeaker()),
        //     rightShooterInterpolation.get(getDistanceToSpeaker()),
        //     angleInterpolation.get(getDistanceToSpeaker()),
        //      getDistanceToSpeaker());
        return new ShootingParameters(2000, 3000, 30, 5);
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
        double angle = getAngleBetween(speakerPose, PoseEstimator.getInstance().getEstimatedRobotPose());
        if (PoseEstimator.getInstance().getEstimatedRobotPose().getY() < point.getY()) {
            angle -=90;
            return Math.abs((180 -getRobotHeading()) - angle) < tolerance;
        } else {
            return Math.abs((270 -getRobotHeading()) - angle) < tolerance;
        }
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
        return distanceTo(speakerPose, PoseEstimator.getInstance().getEstimatedRobotPose());
    }

    public double getSetPointForAline() {
        if (RobotContainer.currentRobotState == RobotConstants.STATIONARY_SHOOTING || RobotContainer.currentRobotState == RobotConstants.PRESET_SHOOTING) {
            return Math.atan2(PoseEstimator.getInstance().getEstimatedRobotPose().getTranslation().getX() - speakerPose.getTranslation().getX(), PoseEstimator.getInstance().getEstimatedRobotPose().getTranslation().getY() - speakerPose.getTranslation().getY());

        } else {
            return Math.atan2(PoseEstimator.getInstance().getEstimatedRobotPose().getTranslation().getX() - feedingPose.getTranslation().getX(), PoseEstimator.getInstance().getEstimatedRobotPose().getTranslation().getY() - feedingPose.getTranslation().getY());
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

    public double distanceTo(Pose2d point1, Pose2d point2) {
        return Math.sqrt(Math.pow(point1.getTranslation().getX() - point2.getTranslation().getX(), 2) + 
        Math.pow(point1.getTranslation().getY() - point2.getTranslation().getY(), 2));
        
    }

    public double getAngleBetween(Pose2d point1, Pose2d point2) {
        return ConvUtil.RadiansToDegrees(Math.atan2(point2.getTranslation().getX() - point1.getTranslation().getX(), point2.getTranslation().getY() - point1.getTranslation().getY()));
    }

    public void update() {
        isNoteLog.update(isNote());
        isInWarmupLog.update(isInWarmUpZone());
        isHeadingForShootingLog.update(isHeadingForShooting());
        isHeadingForFeedingLog.update(isHeadingForFeeding());
        isRobotMovingLog.update(isRobotMoving());
        distanceToSpeakerLog.update(getDistanceToSpeaker());
        robotHeadingLog.update(getRobotHeading());
    }
    
}
