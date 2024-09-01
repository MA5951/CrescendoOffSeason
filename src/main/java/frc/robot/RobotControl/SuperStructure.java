// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotControl;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.RobotConstants;
import frc.robot.Subsystem.Feeder.Feeder;
import frc.robot.Subsystem.Shooter.Shooter;
import frc.robot.Utils.ShootingParameters;

/** Add your docs here. */
public class SuperStructure {

    private static ShootingParameters presetParameters;
    private static ShootingParameters point;
    private static InterpolatingDoubleTreeMap leftShooterInterpolation = new InterpolatingDoubleTreeMap();
    private static InterpolatingDoubleTreeMap rightShooterInterpolation = new InterpolatingDoubleTreeMap();
    private static InterpolatingDoubleTreeMap angleInterpolation = new InterpolatingDoubleTreeMap();
        
    public static void setupInterpolation() {
        for (int i = 0; i < RobotConstants.PointsArry.length; i++) {
            point = RobotConstants.PointsArry[i];
            leftShooterInterpolation.put(point.getLeftSpeed(), point.getDistance());
            rightShooterInterpolation.put(point.getRightSpeed(), point.getDistance());
            angleInterpolation.put(point.getArmAngle(), point.getDistance());
        }
    }

    public static void setPRESETParameters(ShootingParameters parameters) {
        presetParameters = parameters;
    }

    public static ShootingParameters getWarmingParameters() {
        return RobotConstants.WARM_SHOOTING_PARAMETERS;
    }

    public static ShootingParameters getPRESETParameters() {
        return presetParameters;
    }

    public static ShootingParameters getShootingPrameters() {
        return new ShootingParameters(
            leftShooterInterpolation.get(getDistanceToSpeaker()),
            rightShooterInterpolation.get(getDistanceToSpeaker()),
            angleInterpolation.get(getDistanceToSpeaker()),
             getDistanceToSpeaker());
    }

    public static ShootingParameters getFeedingPrameters() {
        return RobotConstants.FEEDING_SHOOTING_PARAMETERS;
    }

    public static double getRobotHeading() {
        return 0;
    }
 
    public static boolean isInWarmUpZone() {
        return false;
    }

    public static boolean isHeading(double angle, double tolerance) {
        return Math.abs(getRobotHeading() - angle) < tolerance;
    }

    public static boolean isHeadingForFeeding() {
        return isHeading(RobotConstants.AngleForFeeding , RobotConstants.FeedingTolerance);
    }

    public static boolean isHeadingForShooting() {
        return isHeading(RobotConstants.AngleFotShooting , RobotConstants.ShootingTolerance);
    }

    public static boolean isRobotMoving() {
        return false;
    }

    public static double getDistanceToSpeaker() {
        return 0d;
    }

    public static boolean isNote() {
        return Feeder.getInstance().isNoteInFeeder() || Shooter.getInstance().isNoteInShooter();
    }

    public static boolean isNoteInFeeder() {
        return Feeder.getInstance().isNoteInFeeder();
    }

    public static boolean isNoteInShooter() {
        return Shooter.getInstance().isNoteInShooter();
    }

    
}
