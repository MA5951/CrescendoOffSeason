// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.StateControl.RobotState;

import com.ma5951.utils.StateControl.StatesTypes.State;
import com.ma5951.utils.StateControl.StatesTypes.StatesConstants;

import edu.wpi.first.wpilibj.DriverStation;

public class RobotStates {

    public static State getRobotEnableState() {
        if (DriverStation.isEnabled()) {
            return StatesConstants.ROBOT_ENABLE;
        } else {
            return StatesConstants.ROBOT_DISABLE;
        }
    }

    public static State getRobotFunctionState() {
        if (DriverStation.isTeleop()) {
            return StatesConstants.TELEOP;
        } else if (DriverStation.isAutonomous()) {
            return StatesConstants.AUTO;
        } else {
            return StatesConstants.TEST;
        }
    }

}