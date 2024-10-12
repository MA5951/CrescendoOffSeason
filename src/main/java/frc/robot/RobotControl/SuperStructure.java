// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotControl;

import com.ma5951.utils.Logger.LoggedBool;
import com.ma5951.utils.Logger.LoggedDouble;
import com.ma5951.utils.Logger.LoggedString;
import com.ma5951.utils.Utils.ConvUtil;
import com.ma5951.utils.Utils.DriverStationUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotConstants;
import frc.robot.Subsystem.Feeder.Feeder;
import frc.robot.Subsystem.PoseEstimation.PoseEstimator;
import frc.robot.Subsystem.PoseEstimation.Vision;
import frc.robot.Subsystem.Shooter.Shooter;
import frc.robot.Subsystem.Swerve.SwerveSubsystem;
import frc.robot.Utils.ShootingParameters;

/** Add your docs here. */
public class SuperStructure {


    private ShootingParameters presetParameters;
    private Pose2d ampPose;
    public boolean isOdometry = false;

    private LoggedBool isNoteLog;
    private LoggedBool isInWarmupLog;
    private LoggedBool isRobotMovingLog;
    private LoggedDouble distanceToSpeakerLog;
    private LoggedDouble robotHeadingLog;
    private LoggedDouble angleToSpaekerLog;
    private LoggedString twodTo3dLog;
        
    public SuperStructure() {
        isNoteLog = new LoggedBool("/SuperStructure/Is Note");
        isInWarmupLog = new LoggedBool("/SuperStructure/Is In Warmup");
        isRobotMovingLog = new LoggedBool("/SuperStructure/Is Robot Moving");
        distanceToSpeakerLog = new LoggedDouble("/SuperStructure/Distance To Speaker");
        robotHeadingLog = new LoggedDouble("/SuperStructure/Robot Heading");
        angleToSpaekerLog = new LoggedDouble("/SuperStructure/Angle To Speaker");
        twodTo3dLog = new LoggedString("/SuperStructure/2D or 3D");
    }

    public void updateAmpPose() {
        ampPose = PoseEstimator.getInstance().getEstimatedRobotPose();
    }

    public boolean shouldCloseArmAfterAmp() {
        return Math.abs(ampPose.getY() - PoseEstimator.getInstance().getEstimatedRobotPose().getY()) >=
         RobotConstants.DISTANCE_TO_CLOSE_ARM;
            

    }

    public void setPRESETParameters(ShootingParameters parameters) {
        presetParameters = parameters;
    }

    
    public ShootingParameters getPRESETParameters() {
        return presetParameters;
    }

    public ShootingParameters getShootingPrameters() {
            return new ShootingParameters(4500, 5000, (
                sample(getDistanceToTag(), RobotConstants.shootingPoses)[0] + 5),//3339: 3 , 5951: 5 / 3.6 / 0, 
                getDistanceToTag()) ;
    }

    public ShootingParameters getFeedingPrameters() {
        return RobotConstants.FEEDING_SHOOTING_PARAMETERS;
    }

    //Cirecle between 0 to 360 //Always positive
    public double getRobotHeading() {
        return Math.abs((SwerveSubsystem.getInstance().getFusedHeading() - SwerveSubsystem.getInstance().getOffsetAngle()) % 360);
    }
 
    public boolean isInWarmUpZone() {
        return getDistanceToTag() < RobotConstants.DISTANCE_TO_WARM;
    }

    public boolean isRobotMoving() {
        return !(SwerveSubsystem.getInstance().getVelocity() < 0.1);
    }

    public double getDistanceToTag() {
        if (Vision.getInstance().isTag() && (Vision.getInstance().getTagID() ==7 || Vision.getInstance().getTagID() ==4)  && Vision.getInstance().getDistance() < 5 &&
        !isOdometry ) { 
            twodTo3dLog.update("2D");
            return Vision.getInstance().getDistance();
        } else {
            isOdometry = true;
            twodTo3dLog.update("3D");
            if (DriverStationUtil.getAlliance() == Alliance.Blue) {
                return PoseEstimator.getInstance().getEstimatedRobotPose().getTranslation().getDistance(RobotConstants.BLUE_SPEAKER.getTranslation()) + 0.04;
            } else if (DriverStation.getAlliance().get() == Alliance.Red) {
                return PoseEstimator.getInstance().getEstimatedRobotPose().getTranslation().getDistance(RobotConstants.RED_SPEAKER.getTranslation()) + 0.04;
            } else {
                return 0;
            }
        }
    }

    public double getSetPointForAline() {
        double xTrget = DriverStationUtil.getAlliance() == Alliance.Red ? 
                RobotConstants.RED_SPEAKER.getX() : RobotConstants.BLUE_SPEAKER.getX();
            double yTrget = RobotConstants.RED_SPEAKER.getY();
            double xDis = Math.abs(PoseEstimator.getInstance().getEstimatedRobotPose().getX() - xTrget);
            double yDis = Math.abs(PoseEstimator.getInstance().getEstimatedRobotPose().getY() - yTrget);
            double angle = Math.atan(yDis / xDis);
            angle = -(angle - Math.PI);
            if (PoseEstimator.getInstance().getEstimatedRobotPose().getY() > yTrget) {
                angle = -angle;
            }
            return angle;
    }

    public double getSetPointForAmpAline() {
            double target = DriverStationUtil.getAlliance() == Alliance.Red ? 
                ConvUtil.DegreesToRadians(-90) : ConvUtil.DegreesToRadians(90);
            return target;
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


    public static double[] sample(double disFromTarget, double[][] arr) {
        final int dis = 0, up = 1, down = 2;
        double[] closestPose = new double[3];
        double minDis = Double.MAX_VALUE;
        int indexOfClosetPose = 0;
        for (int i = 0; i < arr.length; i++) {
            if (minDis > Math.abs(
                disFromTarget -
                arr[i][dis])) {
                closestPose = arr[i];
                minDis = Math.abs(
                    disFromTarget -
                    closestPose[dis]);
                indexOfClosetPose = i;
            }
        }
        if ((indexOfClosetPose == 0 && 
            disFromTarget < closestPose[dis])
            || (indexOfClosetPose == arr.length -1
            && disFromTarget > closestPose[dis])) {
                return new double[] {closestPose[up], closestPose[down]};
        }
        double[] secondCloset = 
            disFromTarget > closestPose[dis]
            ? arr[indexOfClosetPose + 1] :
            arr[indexOfClosetPose - 1];
        double[] smallerDisPose;
        double[] biggerDisPose;
        if (secondCloset[dis] < closestPose[dis]) {
            smallerDisPose = secondCloset;
            biggerDisPose = closestPose;
        } else {
            smallerDisPose = closestPose;
            biggerDisPose = secondCloset;
        }
        double t = 
            (disFromTarget
            - smallerDisPose[dis]) /
            (biggerDisPose[dis] - smallerDisPose[dis]);
        double upV = smallerDisPose[up] + (biggerDisPose[up] - smallerDisPose[up]) * t;
        double downV = smallerDisPose[down] + (biggerDisPose[down] - smallerDisPose[down]) * t;
        return new double[] {upV, downV};
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
        isRobotMovingLog.update(isRobotMoving());
        distanceToSpeakerLog.update(getDistanceToTag());
        robotHeadingLog.update(getRobotHeading());
        angleToSpaekerLog.update(getShootingPrameters().getArmAngle());
    }
    
}
