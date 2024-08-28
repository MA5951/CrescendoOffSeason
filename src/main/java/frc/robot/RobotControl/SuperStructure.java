// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotControl;

import frc.robot.RobotConstants;

/** Add your docs here. */
public class SuperStructure {

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

    public static boolean isNote() {
        return false;
    }

}
