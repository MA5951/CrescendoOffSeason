// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotControl;

/** Add your docs here. */
public class SuperStructure {
    private static SuperStructure superStructure;

    public boolean isInWarmUpZone() {
        return false;
    }

    public boolean isHeadingForShooting() {
        return false;
    }

    public boolean isHeadingForFeeding() {
        return false;
    }

    public boolean isRobotMoving() {
        return false;
    }



    public static SuperStructure getInstance() {
        if (superStructure == null) {
            superStructure = new SuperStructure();
        }
        return superStructure;
    }
}
