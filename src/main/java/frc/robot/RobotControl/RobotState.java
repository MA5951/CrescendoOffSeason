// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotControl;

import com.ma5951.utils.StateControl.StatesTypes.State;

import frc.robot.RobotConstants;

/** Add your docs here. */
public class RobotState {
    public static  RobotState robotStates;
    private static State currentRobotState = RobotConstants.IDLE;
    private static State lastRobotState = RobotConstants.IDLE;

    public void setRobotState(State state) {
        lastRobotState = currentRobotState;
        currentRobotState = state;
    }

    public State getRobotState() {
        return currentRobotState;
    }

    public State getLastRobotState() {
        return lastRobotState;
    }

    public static RobotState getInstance() {
        if (robotStates == null) {
            robotStates = new RobotState();
        }
        return robotStates;
    }
}
